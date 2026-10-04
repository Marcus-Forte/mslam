#include "slam/Slam.hh"
#include "map/VoxelHashMap.hh"
#include "slam/CorrespondenceFinderLogger.hh"
#include "slam/ImuMath.hh"
#include "slam/ImuPreintegration.hh"
#include "slam/Preprocessor.hh"
#include "slam/SensorInput.hh"
#include "slam/SlamServer.hh"
#include "slam/Transform.hh"
#include "slam/registration/ImuRegistration.hh"
#include "slam/registration/PointToPlaneRegistration.hh"
// #include "slam/registration/PointToPointRegistration.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>
#include <vector>

namespace {

constexpr int g_init_scans = 10;
constexpr double g_dense_map_voxel_size = 0.01;
constexpr int g_dense_map_voxel_bucket_size = 10;

// Logs the wall-clock time between construction and destruction.
class ScopedElapsedLogger {
public:
  ScopedElapsedLogger(std::shared_ptr<spdlog::logger> logger, const char *name)
      : logger_(std::move(logger)), name_(name),
        start_(std::chrono::steady_clock::now()) {}

  ~ScopedElapsedLogger() {
    const auto elapsed = std::chrono::steady_clock::now() - start_;
    logger_->info("{} elapsed: {:.3f} ms", name_,
                  std::chrono::duration<double, std::milli>(elapsed).count());
  }

  ScopedElapsedLogger(const ScopedElapsedLogger &) = delete;
  ScopedElapsedLogger &operator=(const ScopedElapsedLogger &) = delete;

private:
  std::shared_ptr<spdlog::logger> logger_;
  const char *name_;
  std::chrono::steady_clock::time_point start_;
};

void logState(const std::shared_ptr<spdlog::logger> &logger,
              const mslam::SlamState &state) {
  logger->info("State: pos=[{:.3f},{:.3f},{:.3f}] rot=[{:.3f},{:.3f},{:.3f}] "
               "vel=[{:.3f},{:.3f},{:.3f}] bg=[{:.4f},{:.4f},{:.4f}] "
               "ba=[{:.4f},{:.4f},{:.4f}]",
               state.position.x(), state.position.y(), state.position.z(),
               state.rotation.x(), state.rotation.y(), state.rotation.z(),
               state.velocity.x(), state.velocity.y(), state.velocity.z(),
               state.gyro_bias.x(), state.gyro_bias.y(), state.gyro_bias.z(),
               state.accel_bias.x(), state.accel_bias.y(),
               state.accel_bias.z());
}

} // namespace
namespace mslam {

std::atomic<bool> Slam::should_stop_{false};

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
  if (config_.map_parameters.dense_map) {
    dense_map_ = std::make_unique<VoxelHashMap>(g_dense_map_voxel_size,
                                                g_dense_map_voxel_bucket_size);
  }
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
  const ScopedElapsedLogger elapsed_logger(logger_, "Slam::Predict");

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

  const Eigen::Matrix3d R = toAffine(0.0, 0.0, 0.0, state_.rotation.x(),
                                     state_.rotation.y(), state_.rotation.z())
                                .linear();

  // Remove the estimated accelerometer bias in the body frame before the
  // helper rotates the specific force into the world frame and removes
  // gravity.
  Eigen::Vector3d world_acceleration =
      toGravityCompensatedWorldAcceleration(state_.rotation, imuData,
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
    state_.rotation.x() = std::atan2(-new_R(1, 2), new_R(2, 2));
    state_.rotation.y() = std::asin(std::clamp(new_R(0, 2), -1.0, 1.0));
    state_.rotation.z() = std::atan2(-new_R(0, 1), new_R(0, 0));
  }

  logger_->debug(
      "Predict stage: IMU preintegration dt: {} s, acc_w: [{}, {}, {}], vel_w: "
      "[{}, {}, {}]",
      delta_between_imu_samples, world_acceleration.x(), world_acceleration.y(),
      world_acceleration.z(), state_.velocity.x(), state_.velocity.y(),
      state_.velocity.z());
  logState(logger_, state_);
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

    // Joint 15-DOF optimization: pose + velocity + biases
    state_ = imu_registration_->Align(current_state, *map_, lidarData.points,
                                      previous_state_, preintegrator_);
    previous_state_ = state_;

    preintegrator_.reset(state_.gyro_bias, state_.accel_bias);
    last_imu_timestamp_ns_.reset();

    logState(logger_, state_);
    return;
  }

  state_ = registration_->Align(state_, *map_, lidarData.points);
  ResetImuPreintegration();
  logger_->debug("Update");
  logState(logger_, state_);
}

mslam::Pose3D Slam::getPose() const {
  return Pose3D{{state_.position.x(), state_.position.y(), state_.position.z(),
                 state_.rotation.x(), state_.rotation.y(),
                 state_.rotation.z()}};
}

Eigen::Affine3d Slam::getTransform() const {
  return toAffine(state_.position.x(), state_.position.y(), state_.position.z(),
                  state_.rotation.x(), state_.rotation.y(),
                  state_.rotation.z());
}

void Slam::startProcessing() {
  running_.store(true);
  logger_->info("SLAM processing started");
}

void Slam::stopProcessing() {
  running_.store(false);
  logger_->info("SLAM processing stopped");
}

void Slam::reset() {
  running_.store(false);
  ResetPose();
  map_->clear();
  if (dense_map_) {
    dense_map_->clear();
  }
  logger_->info("SLAM reset: pose and map cleared");
}

bool Slam::isRunning() const { return running_.load(); }

void Slam::pruneMap() {
  const auto &position = state_.position;
  map_->prune(
      Point{static_cast<float>(position.x()), static_cast<float>(position.y()),
            static_cast<float>(position.z())},
      config_.map_parameters.max_range, config_.map_parameters.max_voxels);
}

void Slam::signalHandler(int signal_number) {
  if (signal_number == SIGINT || signal_number == SIGTERM) {
    should_stop_.store(true);
  }
}

void Slam::run(std::shared_ptr<msensor::ILidar> lidar,
               std::shared_ptr<msensor::IImu> imu, SlamServer &server) {

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  server.updatePose(getPose());
  should_stop_.store(false);
  startProcessing();

  logger_->debug("Using downsample filter: {}",
                 toString(config_.preprocessor.downsample_filter));

  int init_scan_count = 0;

  const bool with_imu = config_.with_imu;
  const bool with_lidar = config_.with_lidar;

  SensorInput sensors(lidar, imu, with_lidar, with_imu);
  Preprocessor preprocessor(config_.preprocessor, logger_);

  while (!should_stop_.load()) {

    if (!running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto scan = sensors.nextScan();

    if (!scan) {
      if (should_stop_.load()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (scan->points.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    std::vector<msensor::IMUData> scan_imu_samples;

    if (with_imu) {
      static uint64_t last_imu_time = 0;

      while (true) {
        if (should_stop_.load()) {
          break;
        }

        const auto imudata = sensors.nextImu();
        if (!imudata.has_value()) {
          break;
        }

        logger_->debug("Processing IMU @ {}, delta {} ms, seq nr {}, "
                       "raw_acc=[{}, {}, {}], raw_gyro=[{}, {}, {}]",
                       imudata->header.timestamp,
                       (imudata->header.timestamp - last_imu_time) * 1e-6,
                       imudata->header.sequence_number, imudata->ax,
                       imudata->ay, imudata->az, imudata->gx, imudata->gy,
                       imudata->gz);

        last_imu_time = imudata->header.timestamp;
        scan_imu_samples.push_back(*imudata);

        Predict(*imudata);
        server.updatePose(getPose());
      }
    }

    if (init_scan_count < g_init_scans) {
      auto filtered_scan = preprocessor.filterNearCenter(*scan);

      auto map_increment = map_->addScan(filtered_scan->points);
      if (dense_map_) {
        dense_map_->addScan(filtered_scan->points);
      }
      pruneMap();
      init_scan_count++;

      server.updateTransformedScan(filtered_scan->points);

      server.updateMapIncrement(map_increment);

      logger_->info("Init scan {}/{}. Map points: {}", init_scan_count,
                    g_init_scans, map_->getPointCloudRepresentation().size());
      ResetImuPreintegration();
      continue;
    }
    if (with_lidar) {
      logger_->debug("Processing Lidar scan with {} points @ {}, seq nr {}",
                     scan->points.size(), scan->header.timestamp,
                     scan->header.sequence_number);

      auto filtered_scan = preprocessor.process(*scan, scan_imu_samples);
      logger_->debug("Preprocess: {} -> {} pts", scan->points.size(),
                     filtered_scan->points.size());

      Update(*filtered_scan);

      server.updatePose(getPose());

      transformCloud(getTransform(), filtered_scan->points);

      auto map_increment = map_->addScan(filtered_scan->points);

      if (dense_map_) {
        dense_map_->addScan(filtered_scan->points);
      }
      pruneMap();

      server.updateTransformedScan(filtered_scan->points);

      server.updateMapIncrement(map_increment);
    }
  }
}

} // namespace mslam
