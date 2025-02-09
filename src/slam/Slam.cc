#include "slam/Slam.hh"

namespace {
void logPose3D(const std::shared_ptr<ILog> &logger, const mslam::Pose3D &pose) {
  logger->log(ILog::Level::INFO, "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}",
              pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}
} // namespace
namespace mslam {

Slam::Slam(const std::shared_ptr<ILog> &logger, const SlamParameters &config,
           const std::shared_ptr<IMap> &map)
    : logger_(logger), config_(config),
      registration_(config.opt_iterations, config.reg_iterations,
                    config.max_correspondence_distance, logger),
      map_(map) {
  ResetPose();
}
void Slam::ResetPose() {
  pose_.setZero();
  logger_->log(ILog::Level::INFO, "Slam Reset: Pose");
}
void Slam::Predict(const msensor::IMUData &imuData) {
  const auto pose_prior = pose_;
  static uint64_t last_timestamp = std::numeric_limits<uint64_t>::max();

  const auto delta = (static_cast<double>(imuData.timestamp) -
                      static_cast<double>(last_timestamp)) *
                     1e-6;

  last_timestamp = imuData.timestamp;

  if (delta < 0) {
    logger_->log(ILog::Level::WARNING, "IMU Loopback detected.");
    return;
  }

  // logger_->log(ILog::Level::DEBUG, "delta: {}", delta);
  pose_[2] = pose_[2] + delta * imuData.gz;

  logger_->log(ILog::Level::INFO, "Predict");
  logPose3D(logger_, pose_);
}
void Slam::Update(const msensor::Scan3D &lidarData) {
  const auto pose_prior = pose_;

  pose_ = registration_.Align3D(pose_prior, *map_, lidarData.points);
  logger_->log(ILog::Level::INFO, "Update");
  logPose3D(logger_, pose_);
}

mslam::Pose3D Slam::getPose() const { return pose_; }
} // namespace mslam