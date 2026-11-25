#include "mslam/slam/registration/IcpWorkspace.hh"

#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"

#include <cstdlib>
#include <cstring>

namespace mslam {

namespace {
// The registrations evaluate each LM trial with a full linear system (moptim's
// speculative linear systems): an accepted trial then costs one evaluation instead
// of two, and x, the iterations and the cost are the same bit for bit.
// Benchmark-only knob: MOPTIM_SPECULATIVE=0 turns it off.
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
  constexpr double kMaxTranslation = 1e-4; // m
  constexpr double kMaxRotation = 1e-4;    // rad
  const Eigen::Matrix<double, 6, 1> step =
      se3Log(Eigen::Affine3d(after * before.inverse()));
  return step.head<3>().norm() < kMaxTranslation &&
         step.tail<3>().norm() < kMaxRotation;
}

IcpWorkspace::IcpWorkspace(
    const IcpParameters &parameters, std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
    std::shared_ptr<const ParallelExecutor> executor,
    std::shared_ptr<moptim::ExecutionContext> context)
    : parameters_(parameters), logger_(std::move(logger)),
      correspondence_finder_(std::move(correspondence_finder)),
      executor_(std::move(executor)),
      context_(context ? std::move(context)
                       : std::make_shared<moptim::ExecutionContext>()) {}

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
  using Clock = std::chrono::steady_clock;
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
  // Assembled in correspondence order.
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
