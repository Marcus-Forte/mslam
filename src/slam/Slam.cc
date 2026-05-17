#include "slam/Slam.hh"
#include "Timer.hh"
#include "slam/CorrespondenceFinder.hh"
#include "slam/PointCloudExporter.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/SlamServer.hh"
#include "slam/Transform.hh"
#include "slam/registration/PointToPlaneRegistration.hh"
#include "slam/registration/PointToPointRegistration.hh"

#include <cmath>
#include <csignal>
#include <limits>
#include <thread>

namespace {

constexpr int g_init_scans = 10;
constexpr double g_gravity_mps2 = 9.80665;
constexpr double g_min_acceleration_norm_mps2 = 1e-3;

mslam::PointCloud toPointCloud3(const mslam::VectorPoint3d &points) {
  mslam::PointCloud point_cloud;
  point_cloud.reserve(points.size());
  for (const auto &point : points) {
    point_cloud.emplace_back(point.x(), point.y(), point.z());
  }
  return point_cloud;
}

void logPose3D(const std::shared_ptr<ILog> &logger, const mslam::Pose3D &pose) {
  logger->log(ILog::Level::INFO,
              "Pose: {:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}", pose[0],
              pose[1], pose[2], pose[3], pose[4], pose[5]);
}

Eigen::Vector3d
toGravityCompensatedWorldAcceleration(const mslam::Pose3D &pose,
                                      const msensor::IMUData &imu_data,
                                      double acceleration_scale) {
  const auto orientation =
      toAffine(0.0, 0.0, 0.0, pose[3], pose[4], pose[5]).linear();
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

Slam::Slam(const std::shared_ptr<ILog> &logger, const SlamConfiguration &config,
           const std::shared_ptr<IMap> &map)
    : logger_(logger), config_(config),
      registration_(std::make_unique<PointToPlaneRegistration>(
          config.parameters.reg_iterations, config.parameters.opt_iterations,
          config.parameters.max_correspondence_distance, logger,
          std::make_shared<CorrespondenceFinder>(logger))),
      map_(map) {
  ResetPose();
}

void Slam::ResetImuPreintegration() {
  imu_velocity_.setZero();
  last_imu_timestamp_ns_.reset();
}

void Slam::ResetPose() {
  pose_.setZero();
  ResetImuPreintegration();
  imu_gravity_aligned_ = false;
  logger_->log(ILog::Level::INFO, "Slam Reset: Pose");
}

bool Slam::TryInitializeGravityAlignment(const msensor::IMUData &imuData) {
  if (imu_gravity_aligned_) {
    return false;
  }

  const auto roll_pitch = estimateGravityAlignedRollPitch(imuData);
  if (!roll_pitch.has_value()) {
    return false;
  }

  pose_[3] = (*roll_pitch).x();
  pose_[4] = (*roll_pitch).y();
  imu_velocity_.setZero();
  imu_gravity_aligned_ = true;

  logger_->log(
      ILog::Level::INFO,
      "Initialized IMU gravity alignment: roll={}, pitch={}, accel_scale={}",
      pose_[3], pose_[4], config_.imu_acceleration_scale);
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
    logger_->log(ILog::Level::WARNING, "IMU Loopback detected.");
    ResetImuPreintegration();
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  if (delta > 1.0) {
    logger_->log(
        ILog::Level::WARNING,
        "Large IMU delta detected: {} seconds. Possible timestamp issue.",
        delta);
    ResetImuPreintegration();
    last_imu_timestamp_ns_ = imuData.header.timestamp;
    return;
  }

  if (just_initialized_gravity) {
    imu_velocity_.setZero();
    return;
  }

  if (!imu_gravity_aligned_) {
    pose_[3] += delta * imuData.gx;
    pose_[4] += delta * imuData.gy;
    pose_[5] += delta * imuData.gz;
    return;
  }

  const Eigen::Vector3d world_acceleration =
      toGravityCompensatedWorldAcceleration(pose_, imuData,
                                            config_.imu_acceleration_scale);

  pose_[0] +=
      delta * imu_velocity_.x() + 0.5 * delta * delta * world_acceleration.x();
  pose_[1] +=
      delta * imu_velocity_.y() + 0.5 * delta * delta * world_acceleration.y();
  pose_[2] +=
      delta * imu_velocity_.z() + 0.5 * delta * delta * world_acceleration.z();

  imu_velocity_ += delta * world_acceleration;
  pose_[3] += delta * imuData.gx;
  pose_[4] += delta * imuData.gy;
  pose_[5] += delta * imuData.gz;

  logger_->log(ILog::Level::DEBUG,
               "IMU preintegration dt: {} s, acc_w: [{}, {}, {}], vel_w: "
               "[{}, {}, {}]",
               delta, world_acceleration.x(), world_acceleration.y(),
               world_acceleration.z(), imu_velocity_.x(), imu_velocity_.y(),
               imu_velocity_.z());
  logger_->log(ILog::Level::DEBUG, "Predict");
  logPose3D(logger_, pose_);
}

void Slam::Update(const Scan &lidarData) {
  const auto pose_prior = pose_;

  pose_ = registration_->Align(pose_prior, *map_, *lidarData.points);
  ResetImuPreintegration();
  logger_->log(ILog::Level::DEBUG, "Update");
  logPose3D(logger_, pose_);
}

mslam::Pose3D Slam::getPose() const { return pose_; }

Eigen::Affine3d Slam::getTransform() const {
  return toAffine(pose_[0], pose_[1], pose_[2], pose_[3], pose_[4], pose_[5]);
}

void Slam::startProcessing() {
  running_.store(true);
  logger_->log(ILog::Level::INFO, "SLAM processing started");
}

void Slam::stopProcessing() {
  running_.store(false);
  logger_->log(ILog::Level::INFO, "SLAM processing stopped");
}

void Slam::reset() {
  running_.store(false);
  ResetPose();
  map_->clear();
  logger_->log(ILog::Level::INFO, "SLAM reset: pose and map cleared");
}

bool Slam::isRunning() const { return running_.load(); }

void Slam::signalHandler(int signal_number) {
  if (signal_number == SIGINT || signal_number == SIGTERM) {
    should_stop_.store(true);
  }
}

void Slam::run(std::shared_ptr<msensor::ILidar> lidar,
               std::shared_ptr<msensor::IImu> imu, SlamServer &server,
               PointCloudExporter &exporter,
               std::shared_ptr<RecordingSensorPlayer> playback_player) {

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  server.updatePose(getPose());
  should_stop_.store(false);
  startProcessing();

  logger_->log(ILog::Level::DEBUG, "Using downsample filter: {}",
               toString(config_.preprocessor.downsample_filter));

  int init_scan_count = 0;
  Timer scan_timer;
  Timer stage_timer;

  Eigen::Affine3d last_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_delta = Eigen::Affine3d::Identity();
  Eigen::Affine3d pre_imu_pose = Eigen::Affine3d::Identity();

  while (!should_stop_.load()) {

    if (!running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto scan = lidar->getScan();

    if (!scan) {
      if (playback_player && playback_player->isFinished()) {
        logger_->log(ILog::Level::INFO,
                     "Playback exhausted; exiting SLAM process.");
        break;
      }
      if (should_stop_.load()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    scan_timer.start();

    if (scan->points->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (config_.with_imu) {
      static uint64_t last_imu_time = 0;
      pre_imu_pose = getTransform();

      while (true) {
        if (should_stop_.load()) {
          break;
        }

        const auto imudata = imu->getImuData();
        if (!imudata.has_value()) {
          break;
        }

        logger_->log(
            ILog::Level::DEBUG,
            "Processing IMU @ {}, delta {} ms, seq nr {}, raw_acc=[{}, {}, "
            "{}], |acc|={}, raw_gyro=[{}, {}, {}]",
            imudata->header.timestamp,
            (imudata->header.timestamp - last_imu_time) * 1e-6,
            imudata->header.sequence_number, imudata->ax, imudata->ay,
            imudata->az,
            std::hypot(imudata->ax, std::hypot(imudata->ay, imudata->az)),
            imudata->gx, imudata->gy, imudata->gz);

        last_imu_time = imudata->header.timestamp;

        stage_timer.start();
        Predict(*imudata);
        const auto imu_predict_us = stage_timer.stop();
        server.updatePose(getPose());

        logger_->log(ILog::Level::DEBUG, "IMU predict took: {} us",
                     imu_predict_us);
      }
    }

    if (init_scan_count < g_init_scans) {
      stage_timer.start();

      auto filtered_scan = removePointsNearCenter(
          *scan, config_.preprocessor.min_distance_to_center);

      exporter.addTransformedScan(*filtered_scan->points);
      stage_timer.start();
      auto map_increment = map_->addScan(*filtered_scan->points);
      const auto map_update_us = stage_timer.stop();
      init_scan_count++;

      server.updateTransformedScan(*filtered_scan->points);
      server.updateMapIncrement(map_increment);
      logger_->log(ILog::Level::INFO, "Init scan {}/{}. Map points: {}",
                   init_scan_count, g_init_scans,
                   map_->getPointCloudRepresentation().size());
      logger_->log(ILog::Level::INFO,
                   "Init timing. addScan: {} us. Total: {} us", map_update_us,
                   scan_timer.stop());
      ResetImuPreintegration();
      continue;
    }
    if (config_.with_lidar) {
      logger_->log(ILog::Level::DEBUG,
                   "Processing Lidar scan with {} points @ {}, seq nr {}",
                   scan->points->size(), scan->header.timestamp,
                   scan->header.sequence_number);

      if (config_.preprocessor.deskew_mode != DeskewMode::Off) {
        stage_timer.start();
        if (config_.preprocessor.deskew_mode == DeskewMode::Imu &&
            config_.with_imu) {
          const Eigen::Affine3d imu_delta =
              pre_imu_pose.inverse() * getTransform();
          scan = deskew(*scan, imu_delta);
        } else {
          scan = deskew(*scan, last_delta);
        }
        const auto deskew_us = stage_timer.stop();
        logger_->log(ILog::Level::DEBUG, "Deskew took: {} us (mode: {})",
                     deskew_us,
                     config_.preprocessor.deskew_mode == DeskewMode::Imu
                         ? "imu"
                         : "constant_velocity");
      }

      const auto pts_in = scan->points->size();

      stage_timer.start();
      auto filtered_scan = removePointsNearCenter(
          *scan, config_.preprocessor.min_distance_to_center);
      const auto range_filter_us = stage_timer.stop();
      logger_->log(ILog::Level::DEBUG,
                   "Range filter:     {:6} -> {:6} pts  ({} us)", pts_in,
                   filtered_scan->points->size(), range_filter_us);

      const auto pts_after_range = filtered_scan->points->size();

      stage_timer.start();
      filtered_scan =
          downsample(*filtered_scan, config_.preprocessor.voxel_size,
                     config_.preprocessor.downsample_filter);
      const auto downsample_us = stage_timer.stop();
      logger_->log(
          ILog::Level::DEBUG, "Downsample ({}): {:6} -> {:6} pts  ({} us)",
          toString(config_.preprocessor.downsample_filter), pts_after_range,
          filtered_scan->points->size(), downsample_us);

      const auto pts_after_downsample = filtered_scan->points->size();

      stage_timer.start();
      filtered_scan =
          filterByIntensity(*filtered_scan, config_.preprocessor.min_intensity);
      const auto intensity_filter_us = stage_timer.stop();
      logger_->log(ILog::Level::DEBUG,
                   "Intensity filter: {:6} -> {:6} pts  ({} us)",
                   pts_after_downsample, filtered_scan->points->size(),
                   intensity_filter_us);

      const auto preprocessor_us =
          range_filter_us + downsample_us + intensity_filter_us;

      stage_timer.start();
      Update(*filtered_scan);
      const auto registration_us = stage_timer.stop();

      const Eigen::Affine3d new_pose = getTransform();
      last_delta = last_pose.inverse() * new_pose;
      last_pose = new_pose;

      stage_timer.start();
      server.updatePose(getPose());

      transformCloud(getTransform(), *filtered_scan->points);
      const auto transform_us = stage_timer.stop();

      exporter.addTransformedScan(*filtered_scan->points);

      stage_timer.start();
      auto map_increment = map_->addScan(*filtered_scan->points);
      const auto add_scan_us = stage_timer.stop();

      stage_timer.start();
      server.updateTransformedScan(*filtered_scan->points);
      server.updateMapIncrement(map_increment);
      const auto transformed_scan_publish_us = stage_timer.stop();

      logger_->log(
          ILog::Level::INFO,
          "Slam timing. Preprocess: {} us. Registration: {} us. "
          "Transform: {} us. addScan: {} us. Publish transformed scan: {} us. "
          "Total: {} us",
          preprocessor_us, registration_us, transform_us, add_scan_us,
          transformed_scan_publish_us, scan_timer.stop());
    }
  }
}

} // namespace mslam