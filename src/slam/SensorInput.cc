#include "mslam/slam/SensorInput.hh"
#include <algorithm>
#include <stdexcept>

namespace mslam {

SensorInput::SensorInput(std::shared_ptr<msensor::ILidar> lidar,
                         std::shared_ptr<msensor::IImu> imu, bool with_lidar,
                         bool with_imu, SensorInputOptions options)
    : options_(options) {
  // A depth of 0 could never hold a sample.
  options_.max_queued_scans =
      std::max<std::size_t>(1, options_.max_queued_scans);
  options_.max_queued_imus = std::max<std::size_t>(1, options_.max_queued_imus);

  // Assign before registering so the destructor unregisters even if the
  // second registration throws.
  lidar_ = std::move(lidar);
  imu_ = std::move(imu);

  const bool block = options_.overflow == SensorOverflow::Block;

  if (with_lidar) {
    if (!lidar_) {
      throw std::invalid_argument("LiDAR sensor is required for live SLAM");
    }
    lidar_->setScanCallback(
        [queues = queues_, block,
         max_scans = options_.max_queued_scans](const Scan &scan) {
          auto queued_scan = std::make_shared<Scan>(scan);
          std::unique_lock lock(queues->mutex);
          if (block) {
            queues->room.wait(lock, [&] {
              return queues->closed || queues->scans.size() < max_scans;
            });
            if (queues->closed) {
              return;
            }
          }
          queues->scans.push_back(std::move(queued_scan));
          if (queues->scans.size() > max_scans) {
            queues->scans.pop_front();
            ++queues->dropped_scans;
          }
        });
  }
  if (with_imu) {
    if (!imu_) {
      throw std::invalid_argument("IMU sensor is required for live SLAM");
    }
    imu_->setImuCallback(
        [queues = queues_, block,
         max_imus = options_.max_queued_imus](const msensor::IMUData &data) {
          std::unique_lock lock(queues->mutex);
          if (block) {
            queues->room.wait(lock, [&] {
              return queues->closed || queues->imus.size() < max_imus;
            });
            if (queues->closed) {
              return;
            }
          }
          queues->imus.push_back(data);
          if (queues->imus.size() > max_imus) {
            queues->imus.pop_front();
          }
        });
  }
}

SensorInput::~SensorInput() {
  // Release a callback that is waiting for room first: unregistering waits for
  // a running callback to return.
  {
    std::lock_guard lock(queues_->mutex);
    queues_->closed = true;
  }
  queues_->room.notify_all();

  if (lidar_) {
    lidar_->setScanCallback({});
  }
  if (imu_) {
    imu_->setImuCallback({});
  }
}

std::shared_ptr<const Scan> SensorInput::nextScan() {
  std::shared_ptr<const Scan> scan;
  {
    std::lock_guard lock(queues_->mutex);
    if (queues_->scans.empty()) {
      return nullptr;
    }
    scan = std::move(queues_->scans.front());
    queues_->scans.pop_front();
  }
  if (options_.overflow == SensorOverflow::Block) {
    queues_->room.notify_all();
  }
  return scan;
}

std::size_t SensorInput::droppedScans() const {
  std::lock_guard lock(queues_->mutex);
  return queues_->dropped_scans;
}

std::optional<msensor::IMUData> SensorInput::nextImu() {
  std::optional<msensor::IMUData> data;
  {
    std::lock_guard lock(queues_->mutex);
    if (queues_->imus.empty()) {
      return std::nullopt;
    }
    data = queues_->imus.front();
    queues_->imus.pop_front();
  }
  if (options_.overflow == SensorOverflow::Block) {
    queues_->room.notify_all();
  }
  return data;
}

} // namespace mslam
