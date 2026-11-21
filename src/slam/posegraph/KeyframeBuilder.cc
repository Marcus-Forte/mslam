#include "mslam/slam/posegraph/KeyframeBuilder.hh"

#include "mslam/slam/posegraph/PoseGraph.hh"

#include <cmath>
#include <stdexcept>

namespace mslam {

KeyframeSelector::KeyframeSelector(double distance, double angle,
                                   double max_interval_s)
    : distance_(distance), angle_(angle),
      max_interval_ns_(static_cast<std::int64_t>(max_interval_s * 1e9)) {}

std::optional<StartTrigger>
KeyframeSelector::startTrigger(std::int64_t stamp_ns,
                               const Eigen::Affine3d &pose,
                               bool use_interval) const {
  if (!has_keyframe_) {
    return StartTrigger::First;
  }
  if ((pose.translation() - pose_.translation()).norm() > distance_) {
    return StartTrigger::Distance;
  }
  const Eigen::Matrix3d relative = pose_.linear().transpose() * pose.linear();
  if (Eigen::AngleAxisd(relative).angle() > angle_) {
    return StartTrigger::Angle;
  }
  if (use_interval && max_interval_ns_ > 0 &&
      stamp_ns - stamp_ns_ >= max_interval_ns_) {
    return StartTrigger::Interval;
  }
  return std::nullopt;
}

void KeyframeSelector::start(std::int64_t stamp_ns,
                             const Eigen::Affine3d &pose) {
  has_keyframe_ = true;
  stamp_ns_ = stamp_ns;
  pose_ = pose;
}

MiniMapBuilder::MiniMapBuilder(double voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
    throw std::invalid_argument("mini map voxel size must be positive");
  }
  inverse_voxel_size_ = 1.0 / voxel_size;
}

void MiniMapBuilder::begin(const Eigen::Affine3d &pose_odom) {
  odom_from_keyframe_ = pose_odom;
  slots_.clear();
  accumulators_.clear();
  scan_count_ = 0;
}

void MiniMapBuilder::add(const PointCloud &points_in_odom) {
  const Eigen::Affine3d keyframe_from_odom = inverseRigid(odom_from_keyframe_);
  for (const auto &point : points_in_odom) {
    const Eigen::Vector3d p =
        keyframe_from_odom * Eigen::Vector3d(point.x, point.y, point.z);
    const Voxel3 voxel(
        static_cast<int>(std::floor(p.x() * inverse_voxel_size_)),
        static_cast<int>(std::floor(p.y() * inverse_voxel_size_)),
        static_cast<int>(std::floor(p.z() * inverse_voxel_size_)));
    const auto [it, inserted] = slots_.try_emplace(
        voxel, static_cast<std::uint32_t>(accumulators_.size()));
    if (inserted) {
      accumulators_.emplace_back();
    }
    auto &acc = accumulators_[it->second];
    acc.x += p.x();
    acc.y += p.y();
    acc.z += p.z();
    acc.intensity += point.intensity;
    ++acc.count;
  }
  ++scan_count_;
}

PointCloud MiniMapBuilder::snapshot() const {
  PointCloud cloud;
  cloud.reserve(accumulators_.size());
  for (const auto &acc : accumulators_) {
    const double inv = 1.0 / static_cast<double>(acc.count);
    Point point;
    point.x = static_cast<float>(acc.x * inv);
    point.y = static_cast<float>(acc.y * inv);
    point.z = static_cast<float>(acc.z * inv);
    point.intensity = static_cast<float>(acc.intensity * inv);
    cloud.push_back(point);
  }
  return cloud;
}

std::shared_ptr<const PointCloud> MiniMapBuilder::seal() {
  auto sealed = std::make_shared<const PointCloud>(snapshot());
  slots_.clear();
  accumulators_.clear();
  scan_count_ = 0;
  return sealed;
}

OwnedMapBuilder::OwnedMapBuilder(double voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
    throw std::invalid_argument("mini map voxel size must be positive");
  }
  inverse_voxel_size_ = 1.0 / voxel_size;
}

void OwnedMapBuilder::begin(const Eigen::Affine3d &pose_odom) {
  odom_from_keyframe_ = pose_odom;
  open_slots_.clear();
  open_.clear();
}

PointCloud OwnedMapBuilder::add(const PointCloud &points_in_odom) {
  const std::size_t first_new = open_.size();
  fresh_.clear();
  for (const auto &point : points_in_odom) {
    const Voxel3 voxel(
        static_cast<int>(
            std::floor(static_cast<double>(point.x) * inverse_voxel_size_)),
        static_cast<int>(
            std::floor(static_cast<double>(point.y) * inverse_voxel_size_)),
        static_cast<int>(
            std::floor(static_cast<double>(point.z) * inverse_voxel_size_)));
    if (sealed_.find(voxel) != sealed_.end()) {
      continue; // owned by an earlier keyframe, for good
    }
    const auto [it, inserted] = open_slots_.try_emplace(
        voxel, static_cast<std::uint32_t>(open_.size()));
    if (inserted) {
      open_.emplace_back();
      fresh_.emplace_back();
    }
    const auto add_to = [&point](Accumulator &acc) {
      acc.x += point.x;
      acc.y += point.y;
      acc.z += point.z;
      acc.intensity += point.intensity;
      ++acc.count;
    };
    // The keyframe's centroid keeps every scan's points; the returned point
    // is only this scan's, and only for a voxel the scan made owned.
    add_to(open_[it->second]);
    if (it->second >= first_new) {
      add_to(fresh_[it->second - first_new]);
    }
  }

  PointCloud owned_now;
  owned_now.reserve(fresh_.size());
  for (const auto &acc : fresh_) {
    const double inv = 1.0 / static_cast<double>(acc.count);
    Point point;
    point.x = static_cast<float>(acc.x * inv);
    point.y = static_cast<float>(acc.y * inv);
    point.z = static_cast<float>(acc.z * inv);
    point.intensity = static_cast<float>(acc.intensity * inv);
    owned_now.push_back(point);
  }
  return owned_now;
}

PointCloud OwnedMapBuilder::snapshot() const {
  const Eigen::Affine3d keyframe_from_odom = inverseRigid(odom_from_keyframe_);
  PointCloud cloud;
  cloud.reserve(open_.size());
  for (const auto &acc : open_) {
    const double inv = 1.0 / static_cast<double>(acc.count);
    const Eigen::Vector3d p =
        keyframe_from_odom *
        Eigen::Vector3d(acc.x * inv, acc.y * inv, acc.z * inv);
    Point point;
    point.x = static_cast<float>(p.x());
    point.y = static_cast<float>(p.y());
    point.z = static_cast<float>(p.z());
    point.intensity = static_cast<float>(acc.intensity * inv);
    cloud.push_back(point);
  }
  return cloud;
}

std::shared_ptr<const PointCloud> OwnedMapBuilder::seal() {
  auto sealed = std::make_shared<const PointCloud>(snapshot());
  for (const auto &entry : open_slots_) {
    sealed_.insert(entry.first); // order irrelevant: a set
  }
  open_slots_.clear();
  open_.clear();
  return sealed;
}

void OwnedMapBuilder::reset() {
  sealed_.clear();
  open_slots_.clear();
  open_.clear();
  fresh_.clear();
}

KeyframeBuilder::KeyframeBuilder(const PoseGraphParameters &parameters)
    : selector_(parameters.keyframe_distance, parameters.keyframe_angle,
                parameters.keyframe_max_interval_s),
      mini_map_(parameters.mini_map_voxel_size),
      owned_(parameters.mini_map_voxel_size) {}

std::optional<SealedKeyframe>
KeyframeBuilder::onScan(std::int64_t stamp_ns,
                        const Eigen::Affine3d &T_odom_scan,
                        const PointCloud &points_in_odom_frame,
                        PointCloud *newly_owned, bool seed_scan) {
  std::optional<SealedKeyframe> sealed;
  if (const auto trigger =
          selector_.startTrigger(stamp_ns, T_odom_scan, !seed_scan)) {
    if (selector_.hasKeyframe()) {
      sealed = sealOpen(*trigger == StartTrigger::Angle ? CloseReason::Angle
                        : *trigger == StartTrigger::Interval
                            ? CloseReason::Interval
                            : CloseReason::Distance);
    }
    selector_.start(stamp_ns, T_odom_scan);
    open_pose_odom_ = T_odom_scan;
    open_stamp_ns_ = stamp_ns;
    mini_map_.begin(T_odom_scan);
    owned_.begin(T_odom_scan); // after the seal: its voxels are frozen now
  }
  mini_map_.add(points_in_odom_frame);
  PointCloud fresh = owned_.add(points_in_odom_frame);
  if (newly_owned != nullptr) {
    *newly_owned = std::move(fresh);
  }
  last_stamp_ns_ = stamp_ns;
  return sealed;
}

SealedKeyframe KeyframeBuilder::sealOpen(CloseReason reason) {
  SealedKeyframe done;
  done.stamp_ns = open_stamp_ns_;
  done.last_stamp_ns = last_stamp_ns_;
  done.close_reason = reason;
  done.pose_odom = open_pose_odom_;
  done.scan_count = mini_map_.scanCount();
  done.view = mini_map_.seal();
  done.mini_map = owned_.seal();
  return done;
}

std::optional<SealedKeyframe> KeyframeBuilder::finish(CloseReason reason) {
  if (!selector_.hasKeyframe()) {
    return std::nullopt;
  }
  SealedKeyframe done = sealOpen(reason);
  selector_.reset();
  return done;
}

void KeyframeBuilder::reset() {
  selector_.reset();
  mini_map_.begin(Eigen::Affine3d::Identity());
  owned_.reset();
  open_pose_odom_ = Eigen::Affine3d::Identity();
  open_stamp_ns_ = 0;
  last_stamp_ns_ = 0;
}

} // namespace mslam
