#pragma once

#include <ostream>
#include <spdlog/logger.h>
#include <stdexcept>
#include <string>

namespace mslam {

enum class MapType { Voxel };
enum class RegistrationMetric3D { PointToPoint, PointToPlane };
enum class DownsampleFilter { VoxelGrid, VoxelHash };
/// How the IMU enters the estimate when `with_imu` is on.
/// PreintegrationFactor: the IMU is a residual of the joint optimization
/// (pose, velocity, biases). DeadReckoning: the IMU is only integrated forward
/// and that pose seeds a pose-only scan registration.
enum class ImuFusion { PreintegrationFactor, DeadReckoning };
/// Which estimator runs. Odometry: LiDAR-inertial odometry only (default).
/// PoseGraph: the same odometry plus a keyframe pose-graph back end with loop
/// closure; see `PoseGraphParameters`.
enum class SlamStrategy { Odometry, PoseGraph };

/// Where the SLAM computes (slam.backend):
///  - Host: the CPU implementation: VoxelHashMap searches, CorrespondenceFinder
///    and NormalEstimator (oneTBB threads), deskewInto() and VoxelFilter,
///    moptim's LevenbergMarquardt on its host backend.
///  - Sycl: the device implementation (DeviceVoxelHashMap with its normal
///    cache, DeviceCorrespondenceSearch, DevicePreprocessor,
///    moptim::DeviceLevenbergMarquardt) on moptim's SYCL backend. Needs a build
///    with MSLAM_WITH_SYCL.
/// Both give the same estimator up to rounding; slam.num_threads applies to
/// both.
enum class ComputeBackend { Host, Sycl };

inline constexpr std::string_view toString(spdlog::level::level_enum level) {
  switch (level) {
  case spdlog::level::trace:
    return "TRACE";
  case spdlog::level::debug:
    return "DEBUG";
  case spdlog::level::info:
    return "INFO";
  case spdlog::level::warn:
    return "WARNING";
  case spdlog::level::err:
    return "ERROR";
  }

  throw std::runtime_error("Invalid log level");
}

/// Parses the `log_level` strings of the JSON configurations.
/// \throws std::runtime_error for any other string.
inline spdlog::level::level_enum parseLogLevel(const std::string &log_level) {
  if (log_level == "trace") {
    return spdlog::level::trace;
  }
  if (log_level == "debug") {
    return spdlog::level::debug;
  }
  if (log_level == "info") {
    return spdlog::level::info;
  }
  if (log_level == "warning") {
    return spdlog::level::warn;
  }
  if (log_level == "error") {
    return spdlog::level::err;
  }

  throw std::runtime_error("Invalid log_level field: " + log_level);
}

inline constexpr std::string_view toString(RegistrationMetric3D metric) {
  switch (metric) {
  case RegistrationMetric3D::PointToPoint:
    return "point_to_point";
  case RegistrationMetric3D::PointToPlane:
    return "point_to_plane";
  }

  throw std::runtime_error("Invalid 3D registration metric");
}

inline constexpr std::string_view toString(DownsampleFilter filter) {
  switch (filter) {
  case DownsampleFilter::VoxelGrid:
    return "voxel_grid";
  case DownsampleFilter::VoxelHash:
    return "voxel_hash";
  }

  throw std::runtime_error("Invalid downsample filter");
}

inline constexpr std::string_view toString(ImuFusion fusion) {
  switch (fusion) {
  case ImuFusion::PreintegrationFactor:
    return "preintegration_factor";
  case ImuFusion::DeadReckoning:
    return "dead_reckoning";
  }

  throw std::runtime_error("Invalid IMU fusion");
}

inline constexpr std::string_view toString(ComputeBackend backend) {
  switch (backend) {
  case ComputeBackend::Host:
    return "host";
  case ComputeBackend::Sycl:
    return "sycl";
  }

  throw std::runtime_error("Invalid compute backend");
}

inline constexpr std::string_view toString(SlamStrategy strategy) {
  switch (strategy) {
  case SlamStrategy::Odometry:
    return "odometry";
  case SlamStrategy::PoseGraph:
    return "pose_graph";
  }

  throw std::runtime_error("Invalid SLAM strategy");
}

/**
 * @brief Algorithm Paramters.
 *
 */
struct SlamParameters {
  unsigned int opt_iterations = 5;
  unsigned int reg_iterations = 5;
  float max_correspondence_distance = 0.1;
  ImuFusion imu_fusion = ImuFusion::PreintegrationFactor; // only with_imu
  // Threads for the correspondence search and normal estimation: 1 runs
  // sequentially, 0 uses every hardware thread. Results do not depend on it.
  unsigned int num_threads = 1;
  ComputeBackend backend = ComputeBackend::Host;
};

/**
 * @brief Pose-graph back end (only used when `strategy` is PoseGraph).
 *
 * The msensor driver already accumulates each scan, so a scan is dense enough
 * and there is no minimum scan count: a keyframe's mini map is built from the
 * scans between it and the next keyframe.
 */
struct PoseGraphParameters {
  // --- Keyframes ---
  // A scan opens a new keyframe when it moved this far from the current one.
  // 0.5 m / 15 deg keeps mini maps compact while consecutive keyframes still
  // overlap strongly (needed by the loop verification).
  double keyframe_distance = 0.5; // m
  double keyframe_angle = 0.26;   // rad (about 15 deg)
  // Close a keyframe after this long even when standing still, bounding the
  // mini map (and the wait for a loop check). 0 = off.
  double keyframe_max_interval_s = 0.0; // s
  // Thinning of a mini map; half the default registration voxel (0.1 m) keeps
  // the detail ICP verification needs at a modest point count.
  double mini_map_voxel_size = 0.05; // m

  // --- Loop closure ---
  // Candidates: keyframes within this distance (map frame) of the new one.
  // Chosen a few times the expected drift over a loop, but small enough to
  // keep scans overlapping.
  double loop_search_radius = 2.0; // m
  // Skip candidates younger than this many keyframes, so neighbours along the
  // trajectory are not mistaken for revisits (10 keyframes = about 5 m).
  unsigned int loop_min_separation = 10; // keyframes
  // The verification target is the candidate plus this many keyframes on each
  // side (a denser, less noisy target than one mini map).
  unsigned int loop_submap_keyframes = 2;
  // Verification gates, the two figures register_scans reports. 0.05 m is
  // about twice the map noise (21 mm RMS measured), 60 % rejects partial or
  // wrong overlaps.
  double loop_max_rms = 0.05;             // m
  double loop_min_matched_fraction = 0.6; // (0, 1]
  // 1-sigma of a loop edge; a verified ICP result is tighter than odometry.
  double loop_sigma_translation = 0.02; // m
  double loop_sigma_rotation = 0.005;   // rad

  // --- Odometry edges ---
  // 1-sigma of the relative pose between consecutive keyframes. Larger than a
  // loop edge so a closure pulls the chain, but not so loose that the graph
  // ignores the odometry.
  double odometry_sigma_translation = 0.05; // m
  double odometry_sigma_rotation = 0.01;    // rad

  // --- Optimizer ---
  unsigned int optimizer_iterations = 20; // Levenberg-Marquardt iterations
  // true: run the back end inline on the SLAM thread so a replay does not
  // depend on timing (bench_slam, integration test); false: worker thread.
  bool synchronous = false;
};

/**
 * @brief sensor data preprocessor
 *
 */
struct PreProcessor {
  float voxel_size = 0.1F;             // input scan voxel downsampling
  float min_distance_to_center = 0.0F; // remove points closer than this
  float min_intensity = 0.0F; // reject points with intensity below this
  DownsampleFilter downsample_filter = DownsampleFilter::VoxelGrid;
  unsigned int points_per_second =
      0;               // lidar scan rate (pts/s), used for deskewing
  bool deskew = false; // on/off: deskew scans using a constant-velocity twist
};

/**
 * @brief MAP Parameters
 *
 */
struct MapParameters {
  float resolution = 0.2;
  unsigned int max_points_per_voxel = 5;
  // Bound on the registration map so per-scan cost does not grow with the
  // session. 0 disables the corresponding bound.
  float max_range = 0.0F; // drop voxels farther than this from the pose
  unsigned int max_voxels = 300000; // hard voxel budget, evict farthest
  bool dense_map = false;           // additionally build a 1 cm dense map
  // Neighbourhood searched around a query's voxel, in voxels (0 = the query's
  // voxel only, 1 = 3x3x3, n = (2n+1)^3). The nearest neighbour is only
  // guaranteed within search_radius * resolution.
  int search_radius = 1;
  /// Largest accepted search_radius (= VoxelHashMap::kMaxAdjacentVoxels).
  static constexpr int kMaxSearchRadius = 8;
};

/**
 * @brief SLAM Configuraton.
 *
 */
struct SlamConfiguration {
  bool with_imu = false;
  bool with_lidar = true;
  double imu_acceleration_scale = 1.0;
  spdlog::level::level_enum log_level = spdlog::level::info;
  MapType map_type = MapType::Voxel;
  std::string remote_scanner = "local";

  PreProcessor preprocessor;
  SlamParameters parameters;
  MapParameters map_parameters;
  SlamStrategy strategy = SlamStrategy::Odometry;
  PoseGraphParameters pose_graph; // used when strategy == PoseGraph

  friend std::ostream &operator<<(std::ostream &os, SlamConfiguration config) {
    os << "IMU: " << config.with_imu << "\n"
       << "LIDAR: " << config.with_lidar << "\n"
       << "IMU Acceleration Scale: " << config.imu_acceleration_scale << "\n"
       << "Log Level: " << toString(config.log_level) << "\n"
       << "Scanner: " << config.remote_scanner << "\n"
       << "# MAP Parameters #" << "\n"
       << "Map Type: "
       << "VoxelHash" << "\n"
       << "Map resolution: " << config.map_parameters.resolution << "\n"
       << "Max pts per voxel: " << config.map_parameters.max_points_per_voxel
       << "\n"
       << "Max map range: " << config.map_parameters.max_range << "\n"
       << "Max map voxels: " << config.map_parameters.max_voxels << "\n"
       << "Dense map: " << config.map_parameters.dense_map << "\n"
       << "Map search radius: " << config.map_parameters.search_radius
       << " voxels\n"
       << "# Preprocessor Parameters #" << "\n"
       << "Voxel Size: " << config.preprocessor.voxel_size << "\n"
       << "Min Distance To Center: "
       << config.preprocessor.min_distance_to_center << "\n"
       << "Downsample Filter: "
       << toString(config.preprocessor.downsample_filter) << "\n"
       << "# SLAM Parameters #" << "\n"
       << "Optimizer Iterations: " << config.parameters.opt_iterations << "\n"
       << "Registration Iterations: " << config.parameters.reg_iterations
       << "\n"
       << "Max Corr. Distance: "
       << config.parameters.max_correspondence_distance << "\n"
       << "IMU Fusion: " << toString(config.parameters.imu_fusion) << "\n"
       << "Threads: " << config.parameters.num_threads << "\n"
       << "Backend: " << toString(config.parameters.backend) << "\n"
       << "Strategy: " << toString(config.strategy) << "\n";
    if (config.strategy == SlamStrategy::PoseGraph) {
      const auto &pg = config.pose_graph;
      os << "# Pose Graph Parameters #" << "\n"
         << "Keyframe distance: " << pg.keyframe_distance << "\n"
         << "Keyframe angle: " << pg.keyframe_angle << "\n"
         << "Keyframe max interval: " << pg.keyframe_max_interval_s << "\n"
         << "Mini map voxel size: " << pg.mini_map_voxel_size << "\n"
         << "Loop search radius: " << pg.loop_search_radius << "\n"
         << "Loop min separation: " << pg.loop_min_separation << "\n"
         << "Loop submap keyframes: " << pg.loop_submap_keyframes << "\n"
         << "Loop max RMS: " << pg.loop_max_rms << "\n"
         << "Loop min matched fraction: " << pg.loop_min_matched_fraction
         << "\n"
         << "Loop sigma translation: " << pg.loop_sigma_translation << "\n"
         << "Loop sigma rotation: " << pg.loop_sigma_rotation << "\n"
         << "Odometry sigma translation: " << pg.odometry_sigma_translation
         << "\n"
         << "Odometry sigma rotation: " << pg.odometry_sigma_rotation << "\n"
         << "Pose graph optimizer iterations: " << pg.optimizer_iterations
         << "\n"
         << "Pose graph synchronous: " << pg.synchronous << "\n";
    }
    return os;
  }
};

class IConfig {
public:
  virtual void load() = 0;
  inline SlamConfiguration getConfig() const { return config_; }

protected:
  SlamConfiguration config_;
};

} // namespace mslam