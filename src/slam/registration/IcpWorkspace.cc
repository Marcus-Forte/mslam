#include "mslam/slam/registration/IcpWorkspace.hh"

#include "mslam/slam/Transform.hh"

namespace mslam {

IcpWorkspace::IcpWorkspace(
    const IcpParameters &parameters, std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
    std::shared_ptr<const ParallelExecutor> executor)
    : parameters_(parameters), logger_(std::move(logger)),
      correspondence_finder_(std::move(correspondence_finder)),
      executor_(std::move(executor)) {}

void IcpWorkspace::begin(const IMap &map, const PointCloud &scan,
                         const Eigen::Affine3d &initial_guess) {
  normal_estimator_.reset(map);
  scan_ = &scan;
  resetSource(initial_guess);
}

void IcpWorkspace::moveSource(const Eigen::Affine3d &delta) {
  transformCloud(delta, source_);
}

void IcpWorkspace::resetSource(const Eigen::Affine3d &total) {
  source_ = *scan_;
  transformCloud(total, source_);
}

void IcpWorkspace::findCorrespondences(const IMap &map) {
  correspondence_finder_->find(
      map, source_, parameters_.max_correspondence_distance, correspondences_);
}

std::size_t IcpWorkspace::findPlaneConstraints(const IMap &map) {
  findCorrespondences(map);

  plane_inputs_.clear();
  plane_offsets_.clear();
  plane_inputs_.reserve(correspondences_.size());
  plane_offsets_.reserve(correspondences_.size());
  normal_estimator_.estimateAll(correspondences_, normal_slots_,
                                executor_.get());
  // Assembled sequentially in correspondence order, as before the normals
  // could be estimated in parallel.
  for (std::size_t i = 0; i < correspondences_.size(); ++i) {
    const auto &[scan_point, map_point] = correspondences_[i];
    const auto &normal = normal_estimator_.normal(normal_slots_[i]);
    if (!normal.has_value()) {
      continue;
    }
    plane_inputs_.emplace_back(scan_point.x, scan_point.y, scan_point.z,
                               normal->x(), normal->y(), normal->z());
    plane_offsets_.push_back(normal->x() * map_point.x +
                             normal->y() * map_point.y +
                             normal->z() * map_point.z);
  }
  logger_->debug("Normal estimation. {} / {} points", plane_offsets_.size(),
                 correspondences_.size());
  return plane_offsets_.size();
}

void IcpWorkspace::attachObserver(
    moptim::LevenbergMarquardt<double> &optimizer) {
  if (!logger_->should_log(spdlog::level::debug)) {
    optimizer.setObserver(nullptr);
    return;
  }
  if (!observer_) {
    observer_.emplace(logger_);
  }
  optimizer.setObserver(&*observer_);
}

} // namespace mslam
