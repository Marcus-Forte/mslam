#pragma once

#include "mslam/config/IConfig.hh"
#include "mslam/map/IMap.hh"
#include "mslam/slam/ISlam.hh"
#include "mslam/slam/ImuPreintegration.hh"
#include "mslam/slam/registration/ImuRegistration.hh"
#include "mslam/slam/registration/PointToPlaneRegistration.hh"
#include "mslam/utils/Parallel.hh"

#include "moptim/ExecutionContext.hh"

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <spdlog/logger.h>

namespace mslam {

/// Cheap counters over the registered scans (a few additions per scan).
struct RegistrationStats {
  std::size_t imu_scans = 0; ///< scans registered with the IMU factor
  /// Scans registered against the map only (IMU disabled, or no IMU data
  /// integrated since the previous scan).
  std::size_t lidar_only_scans = 0;
  /// Scans registered in ImuFusion::DeadReckoning mode: the dead-reckoned pose
  /// seeded a pose-only registration. Counted instead of imu_scans.
  std::size_t dead_reckoning_scans = 0;
  /// IMU-path (either fusion mode) scans whose first registration round found
  /// correspondences, and
  /// the sums of the pose increment that round applied (how far the
  /// IMU-predicted prior was from the registered pose).
  std::size_t first_round_scans = 0;
  double first_round_translation_sum_m = 0.0;
  double first_round_rotation_sum_rad = 0.0;
  /// Sum over the IMU-path scans (either fusion mode) of the time the
  /// preintegrator integrated.
  double preintegrated_seconds = 0.0;

  double meanFirstRoundTranslationMm() const {
    return first_round_scans == 0 ? 0.0
                                  : 1e3 * first_round_translation_sum_m /
                                        static_cast<double>(first_round_scans);
  }
  double meanFirstRoundRotationMrad() const {
    return first_round_scans == 0 ? 0.0
                                  : 1e3 * first_round_rotation_sum_rad /
                                        static_cast<double>(first_round_scans);
  }
};

/// The LiDAR-inertial state estimator: IMU dead reckoning (Predict) and scan
/// to map registration (Update). It does no I/O; SlamPipeline drives it scan by
/// scan and SlamRunner feeds the pipeline from live sensors.
///
/// Not thread-safe: use it from the SLAM thread only.
class Slam : public ISlam {
public:
  Slam(const std::shared_ptr<spdlog::logger> &logger,
       const SlamConfiguration &config, const std::shared_ptr<IMap> &map);
  void ResetPose() override;
  void Predict(const msensor::IMUData &imuData) override;
  void Update(const Scan &lidarData) override;
  Pose3D getPose() const;
  Eigen::Affine3d getTransform() const;

  /// The registration map.
  const std::shared_ptr<IMap> &map() const { return map_; }

  const RegistrationStats &registrationStats() const { return stats_; }

  /// Time spent in the parts of both registrations (search, normals,
  /// constraint assembly, solve), summed over every Update().
  RegistrationTimings registrationTimings() const {
    RegistrationTimings total = registration_->timings();
    total += imu_registration_->timings();
    return total;
  }

  /// The current state (pose, velocity, biases), as of the last Predict or
  /// Update.
  const SlamState &state() const { return state_; }

  /// The pose change the first registration round of the last Update()
  /// applied: how far the prior was from the registered pose. Recorded for
  /// every registered scan, LiDAR-only ones included (the registrationStats()
  /// sums only cover the IMU-path scans). Invalid before the first Update and
  /// when that round found no correspondences.
  const FirstRoundIncrement &lastFirstRound() const {
    return last_first_round_;
  }

  /// Drops the IMU measurements accumulated since the last scan and the
  /// velocity estimate; the next prediction starts from the current pose.
  void ResetImuPreintegration();

private:
  bool TryInitializeGravityAlignment(const msensor::IMUData &imuData);

  // Members are listed in initialization order.
  std::shared_ptr<spdlog::logger> logger_;
  SlamConfiguration config_;
  /// Shared by both registrations and their correspondence finders; null when
  /// slam.num_threads is 1 (sequential).
  std::shared_ptr<const ParallelExecutor> executor_;
  /// The moptim context of both registrations' costs; slam.num_threads sets
  /// its threads too.
  std::shared_ptr<moptim::ExecutionContext> context_;
  std::unique_ptr<PointToPlaneRegistration> registration_;
  std::unique_ptr<ImuRegistration> imu_registration_;
  std::shared_ptr<IMap> map_;
  SlamState state_;
  std::optional<uint64_t> last_imu_timestamp_ns_;
  bool imu_gravity_aligned_ = false;

  // IMU preintegration state
  ImuPreintegrator preintegrator_{ImuNoiseParams{}};
  SlamState previous_state_;
  bool has_previous_state_ = false;
  RegistrationStats stats_;
  FirstRoundIncrement last_first_round_;
};
} // namespace mslam
