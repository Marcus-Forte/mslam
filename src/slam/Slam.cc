#include "slam/Slam.hh"

namespace mslam {

Slam::Slam(const std::shared_ptr<ILog> &logger, const SlamParameters &config,
           const std::shared_ptr<IMap> &map)
    : logger_(logger), config_(config),
      reg_2d_(config.opt_iterations, config.reg_iterations,
              config.max_correspondence_distance, logger),
      map_(map) {
  ResetPose();
}
void Slam::ResetPose() {
  pose_.setZero();
  logger_->log(ILog::Level::INFO, "Slam Reset: Pose -> {}, {}, {}", pose_[0],
               pose_[1], pose_[2]);
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

  logger_->log(ILog::Level::INFO,
               "Slam Predict: Pose update: {},{},{} -> {}, {}, {}",
               pose_prior[0], pose_prior[1], pose_prior[2], pose_[0], pose_[1],
               pose_[2]);
}
void Slam::Update(const msensor::Scan3D &lidarData) {
  const auto pose_prior = pose_;

  pose_ = reg_2d_.Align(pose_prior, *map_, lidarData.points);

  logger_->log(ILog::Level::INFO,
               "Slam Update: Pose update: {},{},{} -> {}, {}, {}",
               pose_prior[0], pose_prior[1], pose_prior[2], pose_[0], pose_[1],
               pose_[2]);
}

mslam::Pose2D Slam::getPose() const { return pose_; }
} // namespace mslam