#pragma once

#include "mslam/config/IConfig.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/SlamPipeline.hh"

#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <spdlog/logger.h>

namespace mslam {

/**
 * @brief The live SLAM loop: pulls scans and IMU samples from the sensors and
 * feeds them to a SlamPipeline.
 *
 * It owns the loop and its control state only. The estimator, the map and
 * the output sink are the pipeline's; stopping is signalled by the caller
 * (see run()), so the runner installs no signal handlers.
 *
 * A scan is paired with the IMU samples of its own time window (see
 * SlamPipeline): the loop feeds every IMU sample to the pipeline as it
 * arrives and holds the scan until the IMU has reached its end, for at most
 * kMaxImuWait. Scans are never reordered; if scans arrive faster than they
 * are processed the sensor queue keeps only the newest one and the dropped
 * scans are counted and logged.
 *
 * Start, stop and reset requests (ISlamControl) may come from any thread. They
 * are applied by the loop between two scans, so a reset never races with a scan
 * being processed.
 */
class SlamRunner : public ISlamControl {
public:
  /// How long a scan waits for the IMU samples up to its end before it is
  /// processed with what has arrived. The data: scans are 100 ms apart (20000
  /// points at 200 kpts/s), the IMU delivers every 5 ms, and the in-process
  /// 16x replay of the integration test delivers a scan's last IMU sample
  /// within a few milliseconds of wall time. Half a scan period is ten IMU
  /// periods of slack, and keeps the loop ahead of the sensors (the scan queue
  /// holds one scan), so a stalled or absent IMU stream delays each scan by
  /// this much at worst, but never blocks it.
  static constexpr std::chrono::milliseconds kMaxImuWait{50};

  SlamRunner(SlamPipeline &pipeline, std::shared_ptr<msensor::ILidar> lidar,
             std::shared_ptr<msensor::IImu> imu,
             const SlamConfiguration &config,
             std::shared_ptr<spdlog::logger> logger);

  /// Runs until @p stop_requested becomes true. May be called from a signal
  /// handler's flag: only an atomic load is done on it.
  void run(const std::atomic<bool> &stop_requested);

  void startProcessing() override;
  void stopProcessing() override;
  void requestReset() override;
  bool isRunning() const { return running_.load(); }

  /// Scans released after kMaxImuWait without their IMU window complete.
  std::size_t imuWaitTimeouts() const { return imu_wait_timeouts_.load(); }

private:
  /// Applies a pending reset request, returns whether there was one. Called by
  /// the loop thread only.
  bool applyPendingReset();

  SlamPipeline &pipeline_;
  std::shared_ptr<msensor::ILidar> lidar_;
  std::shared_ptr<msensor::IImu> imu_;
  bool with_imu_;
  bool with_lidar_;
  std::shared_ptr<spdlog::logger> logger_;
  std::atomic<bool> running_{true};
  std::atomic<bool> reset_requested_{false};
  std::atomic<std::size_t> imu_wait_timeouts_{0};
};

} // namespace mslam
