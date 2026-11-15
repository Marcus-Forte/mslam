#pragma once

#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"
#include "mslam/common/Points.hh"
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>

namespace mslam {

/// What a sensor callback does when its queue is full.
enum class SensorOverflow {
  /// Replace the oldest queued sample. For live sensors: the SLAM stays on the
  /// newest data and the sensor's thread is never held up.
  DropOldest,
  /// Wait until the loop has taken a sample. For a source that can be held
  /// back (a recording): nothing is lost and the source runs at the pace of
  /// the SLAM, so the result does not depend on timing.
  Block,
};

/// Queue depths and overflow policy of a SensorInput.
struct SensorInputOptions {
  std::size_t max_queued_scans = 1;
  std::size_t max_queued_imus = 1000;
  SensorOverflow overflow = SensorOverflow::DropOldest;
};

/**
 * @brief Uniform scan / IMU source for the SLAM loop.
 *
 * Registers callbacks on the live sensors, buffers the latest samples in
 * bounded queues, and unregisters the callbacks on destruction. With
 * SensorOverflow::Block a callback waits for room instead of dropping; the
 * destructor releases a waiting callback (its sample is discarded) before it
 * unregisters, so a blocked source never deadlocks shutdown.
 */
class SensorInput {
public:
  /**
   * @throws std::invalid_argument if a required sensor is missing.
   */
  SensorInput(std::shared_ptr<msensor::ILidar> lidar,
              std::shared_ptr<msensor::IImu> imu, bool with_lidar,
              bool with_imu, SensorInputOptions options = {});
  ~SensorInput();

  SensorInput(const SensorInput &) = delete;
  SensorInput &operator=(const SensorInput &) = delete;

  /// Next available scan, or nullptr if none is ready.
  std::shared_ptr<const Scan> nextScan();
  /// Next available IMU sample, or std::nullopt if none is ready.
  std::optional<msensor::IMUData> nextImu();

  /// Scans that were replaced in the queue before being taken, so far.
  std::size_t droppedScans() const;

private:
  struct Queues {
    std::mutex mutex;
    /// Signalled when a sample is taken or the input is closed (Block only).
    std::condition_variable room;
    std::deque<std::shared_ptr<const Scan>> scans;
    std::deque<msensor::IMUData> imus;
    std::size_t dropped_scans = 0;
    bool closed = false;
  };

  // Sensors whose callbacks this object owns.
  std::shared_ptr<msensor::ILidar> lidar_;
  std::shared_ptr<msensor::IImu> imu_;
  SensorInputOptions options_;
  std::shared_ptr<Queues> queues_ = std::make_shared<Queues>();
};

} // namespace mslam
