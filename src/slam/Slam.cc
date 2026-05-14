#include "slam/Slam.hh"
#include "Timer.hh"
#include "slam/PointCloudExporter.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/SlamServer.hh"
#include "slam/Transform.hh"

#include <csignal>
#include <thread>

namespace {

constexpr int g_init_scans = 10;

mslam::PointCloud3 toPointCloud3(const mslam::VectorPoint3d &points) {
  mslam::PointCloud3 point_cloud;
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
} // namespace
namespace mslam {

std::atomic<bool> Slam::should_stop_{false};

Slam::Slam(const std::shared_ptr<ILog> &logger, const SlamConfiguration &config,
           const std::shared_ptr<IMap> &map)
    : logger_(logger), config_(config), preprocessor_(config.preprocessor),
      registration_(config.parameters.opt_iterations,
                    config.parameters.reg_iterations,
                    config.parameters.max_correspondence_distance, logger),
      map_(map) {
  ResetPose();
}
void Slam::ResetPose() {
  pose_.setZero();
  logger_->log(ILog::Level::INFO, "Slam Reset: Pose");
}
void Slam::Predict(const msensor::IMUData &imuData) {
  static uint64_t last_timestamp = std::numeric_limits<uint64_t>::max();

  const auto delta = (static_cast<double>(imuData.header.timestamp) -
                      static_cast<double>(last_timestamp)) *
                     1e-9;

  last_timestamp = imuData.header.timestamp;

  if (delta < 0) {
    logger_->log(ILog::Level::WARNING, "IMU Loopback detected.");
    return;
  }

  if (delta > 1.0) {
    logger_->log(
        ILog::Level::WARNING,
        "Large IMU delta detected: {} seconds. Possible timestamp issue.",
        delta);
    return;
  }

  logger_->log(ILog::Level::DEBUG, "delta: {} * [{}, {}, {}]", delta,
               imuData.gx, imuData.gy, imuData.gz);
  pose_[3] += delta * imuData.gx;
  pose_[4] += delta * imuData.gy;
  pose_[5] += delta * imuData.gz;

  logger_->log(ILog::Level::DEBUG, "Predict");
  logPose3D(logger_, pose_);
}
void Slam::Update(const msensor::Scan3D &lidarData) {
  const auto pose_prior = pose_;

  pose_ = registration_.Align3D(pose_prior, *map_, *lidarData.points,
                                RegistrationMetric3D::PointToPlane);
  logger_->log(ILog::Level::DEBUG, "Update");
  logPose3D(logger_, pose_);
}

mslam::Pose3D Slam::getPose() const { return pose_; }

Eigen::Affine3d Slam::getTransform() const {
  return toAffine(pose_[0], pose_[1], pose_[2], pose_[3], pose_[4], pose_[5]);
}

const VectorPoint3d &Slam::getLastMapCorrespondences() const {
  return registration_.getLastMapCorrespondences();
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

    const auto scani = lidar->getScan();
    if (!scani) {
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

    msensor::Scan3D scan;
    scan.points->reserve(scani->points->size());
    scan.header = scani->header;
    for (const auto &pt : *scani->points) {
      scan.points->emplace_back(pt.x, pt.y, pt.z);
    }

    if (scan.points->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (config_.with_imu) {
      static uint64_t last_imu_time = 0;
      pre_imu_pose = getTransform();

      while (true) {
        const auto imudata = imu->getImuData();
        if (!imudata.has_value()) {
          break;
        }

        logger_->log(ILog::Level::INFO,
                     "Processing IMU @ {}, delta {} ms, seq nr {}",
                     imudata->header.timestamp,
                     (imudata->header.timestamp - last_imu_time) * 1e-6,
                     imudata->header.sequence_number);

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

      auto filtered_scan = preprocessor_.removePointsNearCenter(scan);
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
      continue;
    }
    if (config_.with_lidar) {
      logger_->log(ILog::Level::INFO,
                   "Processing Lidar scan with {} points @ {}, seq nr {}",
                   scan.points->size(), scan.header.timestamp,
                   scan.header.sequence_number);

      if (config_.preprocessor.deskew_mode != DeskewMode::Off) {
        stage_timer.start();
        std::shared_ptr<msensor::Scan3D> deskewed;
        if (config_.preprocessor.deskew_mode == DeskewMode::Imu &&
            config_.with_imu) {
          const Eigen::Affine3d imu_delta =
              pre_imu_pose.inverse() * getTransform();
          deskewed = preprocessor_.deskewImu(scan, imu_delta);
        } else {
          deskewed = preprocessor_.deskew(scan, last_delta);
        }
        const auto deskew_us = stage_timer.stop();
        logger_->log(ILog::Level::DEBUG, "Deskew took: {} us (mode: {})",
                     deskew_us,
                     config_.preprocessor.deskew_mode == DeskewMode::Imu
                         ? "imu"
                         : "constant_velocity");
        scan = *deskewed;
      }

      stage_timer.start();
      auto filtered_scan = preprocessor_.removePointsNearCenter(scan);
      const auto range_filter_us = stage_timer.stop();

      logger_->log(ILog::Level::DEBUG,
                   "Removed near-center points: {} -> {} points",
                   scan.points->size(), filtered_scan->points->size());

      stage_timer.start();
      filtered_scan = preprocessor_.downsample(*filtered_scan);
      const auto downsample_us = stage_timer.stop();
      const auto preprocessor_us = range_filter_us + downsample_us;

      logger_->log(ILog::Level::DEBUG,
                   "Preprocess benchmark. Range filter: {} us. Downsample: {} "
                   "us",
                   range_filter_us, downsample_us);
      logger_->log(ILog::Level::DEBUG, "Downsampled scan: {} -> {} points",
                   scan.points->size(), filtered_scan->points->size());

      stage_timer.start();
      Update(*filtered_scan);
      const auto registration_us = stage_timer.stop();

      const Eigen::Affine3d new_pose = getTransform();
      last_delta = last_pose.inverse() * new_pose;
      last_pose = new_pose;

      const auto &correspondences = getLastMapCorrespondences();

      stage_timer.start();
      server.updateCorrespondences(toPointCloud3(correspondences));

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