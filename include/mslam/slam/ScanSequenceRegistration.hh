#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/Preprocessor.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"
#include "mslam/slam/registration/PointToPlaneRegistration.hh"
#include "mslam/utils/Parallel.hh"

#include "moptim/ExecutionContext.hh"

#include <cstddef>
#include <memory>
#include <optional>
#include <spdlog/logger.h>

namespace mslam {

struct ScanSequenceParameters {
  float voxel_size = 0.05F;    ///< Every cloud is thinned to voxel centroids.
  float map_resolution = 0.1F; ///< Voxel size of the target map.
  std::size_t max_points_per_voxel = 8;
  IcpParameters icp{30, 5, 0.3F};
  /// Threads of the correspondence search and the normal estimation (1:
  /// sequential, 0: every hardware thread) and of the number of threads moptim
  /// may use for a cost. Results do not depend on it.
  unsigned num_threads = 1;
};

/// What add() did with one cloud.
struct SequenceStep {
  SlamState prior;             ///< Initial pose of the registration.
  bool prior_measured = false; ///< The prior is the caller's measured pose.
  bool registered = false; ///< False for the first cloud (nothing to align to).
  SlamState pose;          ///< Pose of the cloud in the common frame.
  std::size_t input_points = 0;
  std::size_t used_points = 0;     ///< Points left after thinning.
  FirstRoundIncrement first_round; ///< Invalid: no correspondences, prior kept.
  /// Fit of the result against the map before the cloud joined it: thinned
  /// points with a plane constraint, and their point-to-plane RMS (m).
  std::size_t matched_points = 0;
  double rms_m = 0.0;
};

/// Registers whole point clouds one after the other against everything
/// registered so far, by driving PointToPlaneRegistration. The first cloud
/// defines the common frame. The map only grows.
class ScanSequenceRegistration {
public:
  /// Sets the map's neighbour search radius to
  /// ceil(max_correspondence_distance / map_resolution) voxels, clamped to
  /// [1, VoxelHashMap::kMaxAdjacentVoxels] (a warning is logged when clamped),
  /// so the correspondence gate means what it says.
  ScanSequenceRegistration(const ScanSequenceParameters &parameters,
                           std::shared_ptr<spdlog::logger> logger);

  /// Thins @p cloud and registers it against the map. The first cloud is
  /// placed at @p measured_pose, or at identity. A later one starts from
  /// @p measured_pose if given, else from the pose of the previous cloud. In
  /// every case the thinned cloud, moved by the resulting pose, joins the map.
  SequenceStep add(const PointCloud &cloud,
                   const std::optional<Pose3D> &measured_pose = std::nullopt);

  /// Pose of the last cloud added (identity before the first).
  const SlamState &currentPose() const { return pose_; }
  /// Occupied voxels of the map.
  std::size_t mapVoxels() const { return map_.size(); }
  /// Neighbour search radius set on the map, in voxels.
  int searchRadiusVoxels() const { return search_radius_; }

private:
  ScanSequenceParameters parameters_;
  std::shared_ptr<spdlog::logger> logger_;
  VoxelHashMap map_;
  /// Declared before the registrations that use it; null when num_threads is
  /// 1.
  std::shared_ptr<const ParallelExecutor> executor_;
  std::shared_ptr<moptim::ExecutionContext> context_;
  PointToPlaneRegistration registration_;
  IcpWorkspace fit_; ///< Only measures the fit of a result.
  VoxelFilter filter_;
  PointCloud thinned_;
  PointCloud placed_;
  SlamState pose_;
  std::size_t clouds_added_ = 0;
  int search_radius_ = 1;
};

} // namespace mslam
