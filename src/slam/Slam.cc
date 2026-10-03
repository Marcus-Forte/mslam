#include "slam/Slam.hh"
#include "map/VoxelHashMap.hh"
#include "slam/CorrespondenceFinder.hh"
#include "slam/ImuPreintegration.hh"
#include "slam/Preprocessor.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/SlamServer.hh"
#include "slam/Transform.hh"
#include "slam/registration/ImuRegistration.hh"
#include "slam/registration/PointToPlaneRegistration.hh"
// #include "slam/registration/PointToPointRegistration.hh"

#include <cmath>
#include <csignal>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace {

constexpr int g_init_scans = 10;
constexpr double g_gravity_mps2 = 9.80665;
constexpr double g_min_acceleration_norm_mps2 = 1e-3;
constexpr double g_dense_map_voxel_size = 0.01;
constexpr int g_dense_map_voxel_bucket_size = 10;

struct SensorCallbackReset {
  std::shared_ptr<msensor::ILidar> lidar;
  std::shared_ptr<msensor::IImu> imu;

  ~SensorCallbackReset() {
    if (lidar) {
      lidar->setScanCallback({});
    }
    if (imu) {
      imu->setImuCallback({});
    }
  }
};

struct SensorQueues {
  std::mutex mutex;
  std::deque<std::shared_ptr<const mslam::Scan>> scans;
  std::deque<msensor::IMUData> imus;
};

mslam::PointCloud toPointCloud3(const mslam::VectorPoint3d &points) {
  mslam::PointCloud point_cloud;
  point_cloud.reserve(points.size());
  for (const auto &point : points) {
    point_cloud.emplace_back(point.x(), point.y(), point.z());
  }
  return point_cloud;
}

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

Eigen::Vector3d
toGravityCompensatedWorldAcceleration(const Eigen::Vector3d &rotation,
                                      const msensor::IMUData &imu_data,
                                      double acceleration_scale) {
  const auto orientation =
      toAffine(0.0, 0.0, 0.0, rotation.x(), rotation.y(), rotation.z())
          .linear();
  Eigen::Vector3d world_acceleration =
      orientation * (acceleration_scale *
                     Eigen::Vector3d(imu_data.ax, imu_data.ay, imu_data.az));
  world_acceleration.z() -= g_gravity_mps2;
  return world_acceleration;
}

std::optional<Eigen::Vector2d>
estimateGravityAlignedRollPitch(const msensor::IMUData &imu_data) {
  const Eigen::Vector3d body_acceleration(imu_data.ax, imu_data.ay,
                                          imu_data.az);
  const double acceleration_norm = body_acceleration.norm();
  if (!std::isfinite(acceleration_norm) ||
      acceleration_norm < g_min_acceleration_norm_mps2) {
    return std::nullopt;
  }

  const Eigen::Vector3d gravity_direction =
      body_acceleration / acceleration_norm;
  const double roll =
      std::atan2(gravity_direction.y(),
                 std::hypot(gravity_direction.x(), gravity_direction.z()));
  const double pitch =
      std::atan2(-gravity_direction.x(), gravity_direction.z());
  return Eigen::Vector2d(roll, pitch);
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
          std::make_shared<CorrespondenceFinder>(logger))),
      imu_registration_(std::make_unique<ImuRegistration>(
          config.parameters.reg_iterations, config.parameters.opt_iterations,
          config.parameters.max_correspondence_distance, logger,
          std::make_shared<CorrespondenceFinder>(logger))),
      map_(map), dense_map_{std::make_unique<VoxelHashMap>(
                     g_dense_map_voxel_size, g_dense_map_voxel_bucket_size)} {
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
  const bool just_initialized_gravity = TryInitializeGravityAlignment(imuData);

  if (!last_imu_timestamp_ns_.has_value()) {
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  const auto delta = (static_cast<double>(imuData.header.timestamp) -
                      static_cast<double>(*last_imu_timestamp_ns_)) *
                     1e-9;

  last_imu_timestamp_ns_ = imuData.header.timestamp;

  if (delta < 0) {
    logger_->warn("IMU Loopback detected.");
    ResetImuPreintegration();
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  if (delta > 1.0) {
    logger_->warn(
        "Large IMU delta detected: {} seconds. Possible timestamp issue.",
        delta);
    ResetImuPreintegration();
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  if (just_initialized_gravity) {
    state_.velocity.setZero();
    return;
  }

  if (!imu_gravity_aligned_) {
    state_.rotation.x() += delta * imuData.gx;
    state_.rotation.y() += delta * imuData.gy;
    state_.rotation.z() += delta * imuData.gz;
    return;
  }

  // Feed the preintegrator (bias-corrected, body-frame)
  const Eigen::Vector3d gyro(imuData.gx, imuData.gy, imuData.gz);
  const Eigen::Vector3d accel(config_.imu_acceleration_scale * imuData.ax,
                              config_.imu_acceleration_scale * imuData.ay,
                              config_.imu_acceleration_scale * imuData.az);
  preintegrator_.integrate(gyro, accel, delta);

  const Eigen::Vector3d world_acceleration =
      toGravityCompensatedWorldAcceleration(state_.rotation, imuData,
                                            config_.imu_acceleration_scale);

  state_.position +=
      delta * state_.velocity + 0.5 * delta * delta * world_acceleration;

  state_.velocity += delta * world_acceleration;
  state_.rotation.x() += delta * imuData.gx;
  state_.rotation.y() += delta * imuData.gy;
  state_.rotation.z() += delta * imuData.gz;

  logger_->debug("IMU preintegration dt: {} s, acc_w: [{}, {}, {}], vel_w: "
                 "[{}, {}, {}]",
                 delta, world_acceleration.x(), world_acceleration.y(),
                 world_acceleration.z(), state_.velocity.x(),
                 state_.velocity.y(), state_.velocity.z());
  logger_->debug("Predict");
  logState(logger_, state_);
}

void Slam::Update(const Scan &lidarData) {
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

    logger_->debug("Before IMU registration");
    logState(logger_, state_);

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
  logger_->info("SLAM reset: pose and map cleared");
}

bool Slam::isRunning() const { return running_.load(); }

void Slam::signalHandler(int signal_number) {
  if (signal_number == SIGINT || signal_number == SIGTERM) {
    should_stop_.store(true);
  }
}

void Slam::run(std::shared_ptr<msensor::ILidar> lidar,
               std::shared_ptr<msensor::IImu> imu, SlamServer &server,
               std::shared_ptr<RecordingSensorPlayer> playback_player) {

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  server.updatePose(getPose());
  should_stop_.store(false);
  startProcessing();

  logger_->debug("Using downsample filter: {}",
                 toString(config_.preprocessor.downsample_filter));

  int init_scan_count = 0;

  Eigen::Affine3d last_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_delta = Eigen::Affine3d::Identity();
  uint64_t last_scan_timestamp_ns = 0;

  const bool with_imu = config_.with_imu;
  const bool with_lidar = config_.with_lidar;

  auto sensor_queues = std::make_shared<SensorQueues>();
  SensorCallbackReset callback_reset{playback_player ? nullptr : lidar,
                                     playback_player ? nullptr : imu};
  if (!playback_player && with_lidar) {
    if (!lidar) {
      throw std::invalid_argument("LiDAR sensor is required for live SLAM");
    }
    lidar->setScanCallback([sensor_queues](const Scan &scan) {
      auto queued_scan = std::make_shared<Scan>(scan);
      std::lock_guard lock(sensor_queues->mutex);
      sensor_queues->scans.push_back(std::move(queued_scan));
      if (sensor_queues->scans.size() > 1) {
        sensor_queues->scans.pop_front();
      }
    });
  }
  if (!playback_player && with_imu) {
    if (!imu) {
      throw std::invalid_argument("IMU sensor is required for live SLAM");
    }
    imu->setImuCallback([sensor_queues](const msensor::IMUData &data) {
      std::lock_guard lock(sensor_queues->mutex);
      sensor_queues->imus.push_back(data);
      if (sensor_queues->imus.size() > 1000) {
        sensor_queues->imus.pop_front();
      }
    });
  }
  auto nextScan = [&]() -> std::shared_ptr<const Scan> {
    if (playback_player) {
      return playback_player->getScan();
    }
    std::lock_guard lock(sensor_queues->mutex);
    if (sensor_queues->scans.empty()) {
      return nullptr;
    }
    auto scan = std::move(sensor_queues->scans.front());
    sensor_queues->scans.pop_front();
    return scan;
  };
  auto nextImu = [&]() -> std::optional<msensor::IMUData> {
    if (playback_player) {
      return playback_player->getImuData();
    }
    std::lock_guard lock(sensor_queues->mutex);
    if (sensor_queues->imus.empty()) {
      return std::nullopt;
    }
    auto data = sensor_queues->imus.front();
    sensor_queues->imus.pop_front();
    return data;
  };
  Preprocessor preprocessor(config_.preprocessor);

  while (!should_stop_.load()) {

    if (!running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto scan = nextScan();

    if (!scan) {
      if (playback_player && playback_player->isFinished()) {
        logger_->info("Playback exhausted; exiting SLAM process.");
        break;
      }
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

    if (with_imu) {
      static uint64_t last_imu_time = 0;

      while (true) {
        if (should_stop_.load()) {
          break;
        }

        const auto imudata = nextImu();
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

        Predict(*imudata);
        server.updatePose(getPose());
      }
    }

    if (init_scan_count < g_init_scans) {
      auto filtered_scan = preprocessor.filterNearCenter(*scan);

      auto map_increment = map_->addScan(filtered_scan->points);
      dense_map_->addScan(filtered_scan->points);
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

      auto filtered_scan =
          preprocessor.process(*scan, last_delta, last_scan_timestamp_ns);
      logger_->debug("Preprocess: {} -> {} pts", scan->points.size(),
                     filtered_scan->points.size());

      Update(*filtered_scan);

      const Eigen::Affine3d new_pose = getTransform();
      last_delta = last_pose.inverse() * new_pose;
      last_pose = new_pose;
      last_scan_timestamp_ns = scan->header.timestamp;

      server.updatePose(getPose());

      transformCloud(getTransform(), filtered_scan->points);

      auto map_increment = map_->addScan(filtered_scan->points);

      dense_map_->addScan(filtered_scan->points);

      server.updateTransformedScan(filtered_scan->points);

      server.updateMapIncrement(map_increment);
    }
  }
}

} // namespace mslam