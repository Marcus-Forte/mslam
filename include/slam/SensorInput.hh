#pragma once

#include "common/Points.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"
#include <deque>
#include <memory>
#include <mutex>
#include <optional>

namespace mslam {

/**
 * @brief Uniform scan / IMU source for the SLAM loop.
 *
 * Registers callbacks on the live sensors, buffers the latest samples in
 * bounded queues, and unregisters the callbacks on destruction.
 */
class SensorInput {
public:
  /**
   * @throws std::invalid_argument if a required sensor is missing.
   */
  SensorInput(std::shared_ptr<msensor::ILidar> lidar,
              std::shared_ptr<msensor::IImu> imu, bool with_lidar,
              bool with_imu);
  ~SensorInput();

  SensorInput(const SensorInput &) = delete;
  SensorInput &operator=(const SensorInput &) = delete;

  /// Next available scan, or nullptr if none is ready.
  std::shared_ptr<const Scan> nextScan();
  /// Next available IMU sample, or std::nullopt if none is ready.
  std::optional<msensor::IMUData> nextImu();

private:
  struct Queues {
    std::mutex mutex;
    std::deque<std::shared_ptr<const Scan>> scans;
    std::deque<msensor::IMUData> imus;
  };

  // Sensors whose callbacks this object owns.
  std::shared_ptr<msensor::ILidar> lidar_;
  std::shared_ptr<msensor::IImu> imu_;
  std::shared_ptr<Queues> queues_ = std::make_shared<Queues>();
};

} // namespace mslam
