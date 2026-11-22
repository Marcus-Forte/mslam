#include "mslam/slam/registration/IcpWorkspace.hh"

#include "mslam/map/SyclVoxelHashMap.hh"
#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"

#include <cstdlib>
#include <cstring>

namespace mslam {

namespace {
// The registrations evaluate each LM trial with a full linear system (moptim's
// speculative linear systems): an accepted trial then costs one launch instead of
// two, and x, the iterations and the cost are the same bit for bit. Benchmark-only
// knob: MOPTIM_SPECULATIVE=0 turns it off.
bool speculativeLinearSystems() {
  static const bool enabled = [] {
    const char *value = std::getenv("MOPTIM_SPECULATIVE");
    return value == nullptr || std::strcmp(value, "0") != 0;
  }();
  return enabled;
}
} // namespace

bool isNegligibleRoundStep(const Eigen::Affine3d &before,
                           const Eigen::Affine3d &after) {
  constexpr double kMaxTranslation = 1e-6; // m
  constexpr double kMaxRotation = 1e-6;    // rad
  const Eigen::Matrix<double, 6, 1> step =
      se3Log(Eigen::Affine3d(after * before.inverse()));
  return step.head<3>().norm() < kMaxTranslation &&
         step.tail<3>().norm() < kMaxRotation;
}

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

namespace {

/// Work-groups of the device kernels: the executor's threads (1 without one).
std::size_t deviceWidth(const ParallelExecutor *executor) {
  return executor != nullptr ? executor->concurrency() : 1;
}

} // namespace

void IcpWorkspace::findCorrespondences(const IMap &map) {
  if (const SyclVoxelHashMap *device_map = map.deviceMap()) {
    const DeviceCorrespondence *found =
        device_search_.run(*device_map, source_,
                           parameters_.max_correspondence_distance,
                           deviceWidth(executor_.get()));
    correspondences_.clear();
    for (std::size_t i = 0; i < source_.size(); ++i) {
      if (found[i].matched != 0) {
        correspondences_.emplace_back(
            source_[i], Point{found[i].map_point[0], found[i].map_point[1],
                              found[i].map_point[2]});
      }
    }
    return;
  }
  correspondence_finder_->find(
      map, source_, parameters_.max_correspondence_distance, correspondences_);
}

std::size_t IcpWorkspace::findPlaneConstraints(const IMap &map) {
  using Clock = std::chrono::steady_clock;
  if (const SyclVoxelHashMap *device_map = map.deviceMap()) {
    const auto search_start = Clock::now();
    const DeviceCorrespondence *found =
        device_search_.run(*device_map, source_,
                           parameters_.max_correspondence_distance,
                           deviceWidth(executor_.get()));
    const auto assembly_start = Clock::now();
    timings_.search += assembly_start - search_start;

    // The host path's order and arithmetic: correspondences in source order,
    // constraints for those whose map point has a normal. Normals computed by
    // this launch go into the map's cache (several points may share a map
    // point; they computed the same value from the same map).
    correspondences_.clear();
    plane_inputs_.clear();
    plane_offsets_.clear();
    for (std::size_t i = 0; i < source_.size(); ++i) {
      const DeviceCorrespondence &c = found[i];
      if (c.matched == 0) {
        continue;
      }
      const Point &scan_point = source_[i];
      const Point map_point{c.map_point[0], c.map_point[1], c.map_point[2]};
      correspondences_.emplace_back(scan_point, map_point);
      if (c.computed != 0) {
        device_map->storeNormal(c.slot,
                                c.has_normal != 0 ? c.normal : nullptr);
        ++timings_.normals_computed;
      } else {
        ++timings_.normals_cached;
      }
      if (c.has_normal == 0) {
        continue;
      }
      plane_inputs_.emplace_back(scan_point.x, scan_point.y, scan_point.z,
                                 c.normal[0], c.normal[1], c.normal[2]);
      plane_offsets_.push_back(c.normal[0] * map_point.x +
                               c.normal[1] * map_point.y +
                               c.normal[2] * map_point.z);
    }
    timings_.assembly += Clock::now() - assembly_start;
    logger_->debug("Normal estimation. {} / {} points", plane_offsets_.size(),
                   correspondences_.size());
    return plane_offsets_.size();
  }

  const auto search_start = Clock::now();
  findCorrespondences(map);
  const auto normals_start = Clock::now();
  timings_.search += normals_start - search_start;

  plane_inputs_.clear();
  plane_offsets_.clear();
  plane_inputs_.reserve(correspondences_.size());
  plane_offsets_.reserve(correspondences_.size());
  normal_estimator_.estimateAll(correspondences_, normal_slots_,
                                executor_.get());
  const auto assembly_start = Clock::now();
  timings_.normals += assembly_start - normals_start;
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
  timings_.assembly += Clock::now() - assembly_start;
  logger_->debug("Normal estimation. {} / {} points", plane_offsets_.size(),
                 correspondences_.size());
  return plane_offsets_.size();
}

bool IcpWorkspace::useDeviceOptimizer() const {
  static const bool host_lm = [] {
    const char *value = std::getenv("MSLAM_HOST_LM");
    return value != nullptr && std::strcmp(value, "1") == 0;
  }();
  return !host_lm && !logger_->should_log(spdlog::level::debug);
}

void IcpWorkspace::attachObserver(
    moptim::LevenbergMarquardt<double> &optimizer) {
  optimizer.setSpeculativeLinearSystems(speculativeLinearSystems());
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
