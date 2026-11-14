#include "mslam/slam/SlamRunner.hh"

#include "mslam/slam/SensorInput.hh"

#include <chrono>
#include <thread>
#include <vector>

namespace mslam {

SlamRunner::SlamRunner(SlamPipeline &pipeline,
                       std::shared_ptr<msensor::ILidar> lidar,
                       std::shared_ptr<msensor::IImu> imu,
                       const SlamConfiguration &config,
                       std::shared_ptr<spdlog::logger> logger)
    : pipeline_(pipeline), lidar_(std::move(lidar)), imu_(std::move(imu)),
      with_imu_(config.with_imu), with_lidar_(config.with_lidar),
      logger_(std::move(logger)) {}

void SlamRunner::startProcessing() {
  running_.store(true);
  logger_->info("SLAM processing started");
}

void SlamRunner::stopProcessing() {
  running_.store(false);
  logger_->info("SLAM processing stopped");
}

void SlamRunner::requestReset() {
  // Stop right away so no further scan is taken in the meantime; the state
  // itself is cleared by the loop thread, between scans.
  running_.store(false);
  reset_requested_.store(true);
}

bool SlamRunner::applyPendingReset() {
  if (reset_requested_.exchange(false)) {
    pipeline_.reset();
    return true;
  }
  return false;
}

void SlamRunner::run(const std::atomic<bool> &stop_requested) {
  using namespace std::chrono_literals;
  using Clock = std::chrono::steady_clock;

  pipeline_.publishPose();
  running_.store(true);
  logger_->info("SLAM processing started");

  SensorInput sensors(lidar_, imu_, with_lidar_, with_imu_);
  // The scan being held until its IMU window is complete.
  std::shared_ptr<const Scan> held_scan;
  Clock::time_point held_since;
  std::size_t reported_drops = 0;

  while (!stop_requested.load()) {
    if (applyPendingReset()) {
      held_scan.reset(); // belongs to the state that was just cleared
    }

    if (!running_.load()) {
      held_since = Clock::now(); // the wait starts when processing resumes
      std::this_thread::sleep_for(10ms);
      continue;
    }

    if (with_imu_) {
      while (const auto imu_sample = sensors.nextImu()) {
        pipeline_.pushImu(*imu_sample);
      }
    }

    if (const auto drops = sensors.droppedScans(); drops != reported_drops) {
      logger_->warn("Dropped {} scan(s) that were not processed in time ({} "
                    "in total)",
                    drops - reported_drops, drops);
      reported_drops = drops;
    }

    if (!held_scan) {
      auto scan = sensors.nextScan();
      if (!scan || scan->points.empty()) {
        std::this_thread::sleep_for(1ms);
        continue;
      }
      held_scan = std::move(scan);
      held_since = Clock::now();
    }

    if (!pipeline_.imuCovers(*held_scan)) {
      if (Clock::now() - held_since < kMaxImuWait) {
        std::this_thread::sleep_for(500us);
        continue;
      }
      ++imu_wait_timeouts_;
      logger_->warn("IMU data for the scan @ {} did not arrive within {} ms; "
                    "processing it with what is there",
                    held_scan->header.timestamp, kMaxImuWait.count());
    }

    pipeline_.processSyncedScan(*held_scan);
    held_scan.reset();
  }
}

} // namespace mslam
