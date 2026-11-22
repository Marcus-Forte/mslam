#include "mslam/slam/ScanSequenceRegistration.hh"

#include "mslam/slam/CorrespondenceFinderLogger.hh"
#include "mslam/slam/MapFactory.hh"
#include "mslam/slam/Transform.hh"

#include <algorithm>
#include <cmath>

namespace mslam {

namespace {

SlamState stateFromPose(const Pose3D &pose) {
  SlamState state;
  state.position = pose.head<3>().transpose();
  state.rotation = pose.tail<3>().transpose();
  return state;
}

} // namespace

ScanSequenceRegistration::ScanSequenceRegistration(
    const ScanSequenceParameters &parameters,
    std::shared_ptr<spdlog::logger> logger)
    : parameters_(parameters), logger_(logger),
      map_(makeVoxelHashMap(parameters.map_resolution,
                            parameters.max_points_per_voxel, parameters.backend)),
      registration_(parameters.icp.registration_iterations,
                    parameters.icp.optimizer_iterations,
                    parameters.icp.max_correspondence_distance, logger,
                    createLoggingCorrespondenceFinder(logger),
                    parameters.backend, parameters.sycl_threads),
      fit_(parameters.icp, logger, createLoggingCorrespondenceFinder(logger),
           parameters.backend, parameters.sycl_threads) {
  // The tolerance keeps a float quotient such as 0.3F / 0.1F (3.00000004) from
  // costing an extra ring of voxels.
  const double wanted = std::ceil(
      static_cast<double>(parameters.icp.max_correspondence_distance) /
      static_cast<double>(parameters.map_resolution) * (1.0 - 1e-6));
  const double clamped = std::clamp(
      wanted, 1.0, static_cast<double>(VoxelHashMap::kMaxAdjacentVoxels));
  search_radius_ = static_cast<int>(clamped);
  map_->setNumAdjacentVoxelSearch(search_radius_);
  logger_->info("map search radius: {} voxels of {} m", search_radius_,
                parameters.map_resolution);
  if (clamped != wanted) {
    logger_->warn("max_correspondence_distance {} m needs {} voxels of {} m; "
                  "clamped to {}, so the effective gate is about {} m",
                  parameters.icp.max_correspondence_distance, wanted,
                  parameters.map_resolution, search_radius_,
                  search_radius_ * parameters.map_resolution);
  }
}

SequenceStep
ScanSequenceRegistration::add(const PointCloud &cloud,
                              const std::optional<Pose3D> &measured_pose) {
  SequenceStep step;
  step.input_points = cloud.size();
  filter_.apply(cloud, -1.0F, parameters_.voxel_size,
                DownsampleFilter::VoxelGrid, 0.0F, thinned_);
  step.used_points = thinned_.size();

  step.registered = clouds_added_ > 0;
  step.prior_measured = measured_pose.has_value();
  step.prior = measured_pose ? stateFromPose(*measured_pose)
                             : (step.registered ? pose_ : SlamState{});

  if (step.registered) {
    step.pose = registration_.Align(step.prior, *map_, thinned_);
    step.first_round = registration_.lastFirstRound();

    // Fit of the result, measured against the map as it was for the alignment.
    fit_.begin(*map_, thinned_, toAffine(step.pose));
    step.matched_points = fit_.findPlaneConstraints(*map_);
    double sum_squares = 0.0;
    const auto inputs = fit_.planeInputs();
    const auto offsets = fit_.planeOffsets();
    for (std::size_t i = 0; i < step.matched_points; ++i) {
      const double residual =
          offsets[i] - inputs[i].tail<3>().dot(inputs[i].head<3>());
      sum_squares += residual * residual;
    }
    if (step.matched_points > 0) {
      step.rms_m =
          std::sqrt(sum_squares / static_cast<double>(step.matched_points));
    }
  } else {
    step.pose = step.prior;
  }

  placed_ = thinned_;
  const Eigen::Affine3d placement = toAffine(step.pose);
  transformCloud(placement, placed_);
  map_->addScan(placed_);

  pose_ = step.pose;
  ++clouds_added_;
  return step;
}

} // namespace mslam
