#include "slam/SensorInput.hh"
#include "slam/RecordingSensorPlayer.hh"
#include <stdexcept>

namespace mslam {

namespace {
constexpr std::size_t k_max_queued_scans = 1;
constexpr std::size_t k_max_queued_imus = 1000;
} // namespace

SensorInput::SensorInput(std::shared_ptr<msensor::ILidar> lidar,
                         std::shared_ptr<msensor::IImu> imu,
                         std::shared_ptr<RecordingSensorPlayer> playback_player,
                         bool with_lidar, bool with_imu)
    : playback_player_(std::move(playback_player)) {
  if (playback_player_) {
    return;
  }

  // Assign before registering so the destructor unregisters even if the
  // second registration throws.
  lidar_ = std::move(lidar);
  imu_ = std::move(imu);

  if (with_lidar) {
    if (!lidar_) {
      throw std::invalid_argument("LiDAR sensor is required for live SLAM");
    }
    lidar_->setScanCallback([queues = queues_](const Scan &scan) {
      auto queued_scan = std::make_shared<Scan>(scan);
      std::lock_guard lock(queues->mutex);
      queues->scans.push_back(std::move(queued_scan));
      if (queues->scans.size() > k_max_queued_scans) {
        queues->scans.pop_front();
      }
    });
  }
  if (with_imu) {
    if (!imu_) {
      throw std::invalid_argument("IMU sensor is required for live SLAM");
    }
    imu_->setImuCallback([queues = queues_](const msensor::IMUData &data) {
      std::lock_guard lock(queues->mutex);
      queues->imus.push_back(data);
      if (queues->imus.size() > k_max_queued_imus) {
        queues->imus.pop_front();
      }
    });
  }
}

SensorInput::~SensorInput() {
  if (lidar_) {
    lidar_->setScanCallback({});
  }
  if (imu_) {
    imu_->setImuCallback({});
  }
}

std::shared_ptr<const Scan> SensorInput::nextScan() {
  if (playback_player_) {
    return playback_player_->getScan();
  }
  std::lock_guard lock(queues_->mutex);
  if (queues_->scans.empty()) {
    return nullptr;
  }
  auto scan = std::move(queues_->scans.front());
  queues_->scans.pop_front();
  return scan;
}

std::optional<msensor::IMUData> SensorInput::nextImu() {
  if (playback_player_) {
    return playback_player_->getImuData();
  }
  std::lock_guard lock(queues_->mutex);
  if (queues_->imus.empty()) {
    return std::nullopt;
  }
  auto data = queues_->imus.front();
  queues_->imus.pop_front();
  return data;
}

} // namespace mslam
