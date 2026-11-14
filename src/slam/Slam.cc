#include "mslam/slam/Slam.hh"
#include "mslam/slam/CorrespondenceFinderLogger.hh"
#include "mslam/slam/ImuMath.hh"
#include "mslam/slam/ImuPreintegration.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/ImuRegistration.hh"
#include "mslam/slam/registration/PointToPlaneRegistration.hh"
#include "mslam/utils/Timing.hh"

#include <cmath>

namespace {

void logState(const std::shared_ptr<spdlog::logger> &logger,
              const mslam::SlamState &state,
              spdlog::level::level_enum level = spdlog::level::info) {
  logger->log(level,
              "State: pos=[{:.3f},{:.3f},{:.3f}] rot=[{:.3f},{:.3f},{:.3f}] "
              "vel=[{:.3f},{:.3f},{:.3f}] bg=[{:.4f},{:.4f},{:.4f}] "
              "ba=[{:.4f},{:.4f},{:.4f}]",
              state.position.x(), state.position.y(), state.position.z(),
              state.rotation.x(), state.rotation.y(), state.rotation.z(),
              state.velocity.x(), state.velocity.y(), state.velocity.z(),
              state.gyro_bias.x(), state.gyro_bias.y(), state.gyro_bias.z(),
              state.accel_bias.x(), state.accel_bias.y(), state.accel_bias.z());
}

} // namespace
namespace mslam {

Slam::Slam(const std::shared_ptr<spdlog::logger> &logger,
           const SlamConfiguration &config, const std::shared_ptr<IMap> &map)
    : logger_(logger), config_(config),
      registration_(std::make_unique<PointToPlaneRegistration>(
          config.parameters.reg_iterations, config.parameters.opt_iterations,
          config.parameters.max_correspondence_distance, logger,
          createLoggingCorrespondenceFinder(logger))),
      imu_registration_(std::make_unique<ImuRegistration>(
          config.parameters.reg_iterations, config.parameters.opt_iterations,
          config.parameters.max_correspondence_distance, logger,
          createLoggingCorrespondenceFinder(logger))),
      map_(map) {
  ResetPose();
}

void Slam::ResetImuPreintegration() {
  state_.velocity.setZero();
  last_imu_timestamp_ns_.reset();
  preintegrator_.reset(previous_state_.gyro_bias, previous_state_.accel_bias);
  has_previous_state_ = false;
  logger_->warn("Reset IMU preintegration");
}

void Slam::ResetPose() {
  state_ = SlamState{};
  ResetImuPreintegration();
  imu_gravity_aligned_ = false;
  logger_->info("Slam Reset: Pose");
}

bool Slam::TryInitializeGravityAlignment(const msensor::IMUData &imuData) {
  if (imu_gravity_aligned_) {
    return false;
  }

  const auto roll_pitch = estimateGravityAlignedRollPitch(imuData);
  if (!roll_pitch.has_value()) {
    return false;
  }

  state_.rotation.x() = (*roll_pitch).x();
  state_.rotation.y() = (*roll_pitch).y();
  state_.velocity.setZero();
  imu_gravity_aligned_ = true;

  logger_->info(
      "Initialized IMU gravity alignment: roll={}, pitch={}, accel_scale={}",
      state_.rotation.x(), state_.rotation.y(), config_.imu_acceleration_scale);
  return true;
}

void Slam::Predict(const msensor::IMUData &imuData) {
  // Called at the IMU rate: timed and logged at debug level only.
  const ScopedElapsedLogger elapsed_logger(logger_, "Slam::Predict",
                                           spdlog::level::debug);

  const bool just_initialized_gravity = TryInitializeGravityAlignment(imuData);

  if (!last_imu_timestamp_ns_.has_value()) {
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  const auto delta_between_imu_samples =
      (static_cast<double>(imuData.header.timestamp) -
       static_cast<double>(*last_imu_timestamp_ns_)) *
      1e-9;

  last_imu_timestamp_ns_ = imuData.header.timestamp;

  if (delta_between_imu_samples < 0) {
    logger_->warn("IMU Loopback detected.");
    ResetImuPreintegration();
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  if (delta_between_imu_samples > 1.0) {
    logger_->warn(
        "Large IMU delta detected: {} seconds. Possible timestamp issue.",
        delta_between_imu_samples);
    ResetImuPreintegration();
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  if (just_initialized_gravity) {
    state_.velocity.setZero();
    return;
  }

  if (!imu_gravity_aligned_) {
    state_.rotation.x() += delta_between_imu_samples * imuData.gx;
    state_.rotation.y() += delta_between_imu_samples * imuData.gy;
    state_.rotation.z() += delta_between_imu_samples * imuData.gz;
    return;
  }

  // Feed the preintegrator (bias-corrected, body-frame)
  const Eigen::Vector3d gyro(imuData.gx, imuData.gy, imuData.gz);
  const Eigen::Vector3d accel(config_.imu_acceleration_scale * imuData.ax,
                              config_.imu_acceleration_scale * imuData.ay,
                              config_.imu_acceleration_scale * imuData.az);
  preintegrator_.integrate(gyro, accel, delta_between_imu_samples);

  const Eigen::Matrix3d R = eulerToRotation(state_.rotation);

  // Remove the estimated accelerometer bias in the body frame before the
  // helper rotates the specific force into the world frame and removes
  // gravity.
  Eigen::Vector3d world_acceleration =
      gravityCompensatedWorldAcceleration(R, imuData,
                                          config_.imu_acceleration_scale) -
      R * state_.accel_bias;

  state_.position += delta_between_imu_samples * state_.velocity +
                     0.5 * delta_between_imu_samples *
                         delta_between_imu_samples * world_acceleration;

  state_.velocity += delta_between_imu_samples * world_acceleration;

  const Eigen::Vector3d unbiased_gyro = gyro - state_.gyro_bias;
  const double angle = unbiased_gyro.norm() * delta_between_imu_samples;
  if (angle > 0.0) {
    const Eigen::Matrix3d dR =
        Eigen::AngleAxisd(angle, unbiased_gyro.normalized()).toRotationMatrix();
    const Eigen::Matrix3d new_R = R * dR;
    state_.rotation = rotationToEuler(new_R);
  }

  logger_->debug(
      "Predict stage: IMU preintegration dt: {} s, acc_w: [{}, {}, {}], vel_w: "
      "[{}, {}, {}]",
      delta_between_imu_samples, world_acceleration.x(), world_acceleration.y(),
      world_acceleration.z(), state_.velocity.x(), state_.velocity.y(),
      state_.velocity.z());
  if (logger_->should_log(spdlog::level::debug)) {
    logState(logger_, state_, spdlog::level::debug);
  }
}

void Slam::Update(const Scan &lidarData) {
  const ScopedElapsedLogger elapsed_logger(logger_, "Slam::Update");

  if (config_.with_imu && preintegrator_.deltaTime() > 0.0) {
    // Initialize previous state on first call
    if (!has_previous_state_) {
      previous_state_ = state_;
      has_previous_state_ = true;
    }

    // Use current dead-reckoned state as initial guess, carry previous biases
    SlamState current_state = state_;
    current_state.velocity = previous_state_.velocity;
    current_state.gyro_bias = previous_state_.gyro_bias;
    current_state.accel_bias = previous_state_.accel_bias;

    stats_.preintegrated_seconds += preintegrator_.deltaTime();

    // Joint 15-DOF optimization: pose + velocity + biases
    state_ = imu_registration_->Align(current_state, *map_, lidarData.points,
                                      previous_state_, preintegrator_);
    previous_state_ = state_;

    ++stats_.imu_scans;
    if (const auto &first = imu_registration_->lastFirstRound(); first.valid) {
      ++stats_.first_round_scans;
      stats_.first_round_translation_sum_m += first.translation_m;
      stats_.first_round_rotation_sum_rad += first.rotation_rad;
    }

    // Only the preintegrator restarts at the scan. The IMU clock keeps
    // running: the interval between the last sample before this scan and the
    // first one after it belongs to the next scan and must be integrated.
    // (The LiDAR-only path below does reset the clock: there the state was
    // re-anchored on the map and the velocity dropped on purpose.)
    preintegrator_.reset(state_.gyro_bias, state_.accel_bias);

    logState(logger_, state_);
    return;
  }

  state_ = registration_->Align(state_, *map_, lidarData.points);
  ++stats_.lidar_only_scans;
  ResetImuPreintegration();
  logger_->debug("Update");
  logState(logger_, state_);
}

mslam::Pose3D Slam::getPose() const {
  return Pose3D{{state_.position.x(), state_.position.y(), state_.position.z(),
                 state_.rotation.x(), state_.rotation.y(),
                 state_.rotation.z()}};
}

Eigen::Affine3d Slam::getTransform() const { return toAffine(state_); }

} // namespace mslam
