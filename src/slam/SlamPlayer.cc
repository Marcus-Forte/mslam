#include "slam/SlamPlayer.hh"
#include "ConsoleLogger.hh"
#include "slam/Slam.hh"
#include "timing/timing.hh"
#include <thread>

namespace mslam {

const int g_init_scans = 5;

SlamPlayer::SlamPlayer(const std::filesystem::path &file,
                       const std::shared_ptr<ILog> &logger,
                       const SlamConfiguration &config,
                       const std::shared_ptr<IMap> &map)
    : player_(file), logger_(logger), config_(config), map_(map) {}

void SlamPlayer::run() {

  auto slam_logger = std::make_shared<ConsoleLogger>();
  slam_logger->setLevel(ILog::Level::INFO);
  mslam::Slam slam(slam_logger, config_.parameters, map_);

  int init_scan_count = 0;

  while (player_.next()) {

    const auto time_start = timing::getNowUs();

    const auto &entry = player_.getLastEntry();

    if (entry.entry_case() == sensors::RecordingEntry::kImu) {
      const auto imudata = fromEntryToImu(entry);
      if (!config_.with_imu) {
        continue;
      }
      slam.Predict(imudata);

    } else if (entry.entry_case() == sensors::RecordingEntry::kScan) {
      const auto scan = fromEntryScan3D(entry);
      if (!config_.with_lidar) {
        continue;
      }

      if (init_scan_count < g_init_scans) {
        map_->addScan(scan.points);
        init_scan_count++;
        logger_->log(ILog::Level::INFO, "Init scan {}/{}. Map points: {}",
                     init_scan_count, g_init_scans,
                     map_->getPointCloudRepresentation().size());
        continue;
      }

      slam.Update(scan);
    }

    const auto delta_time = timing::getNowUs() - time_start;
    const auto time_delay =
        config_.player_config.entry_delay_ms * 1000 - delta_time;

    /// \todo fix for imu and lidar updates at any time.
    if (time_delay > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(time_delay));
    }
  }

  std::cout << "Finished Slam player." << std::endl;
}

msensor::IMUData
SlamPlayer::fromEntryToImu(const sensors::RecordingEntry &entry) {
  return {entry.imu().ax(),       entry.imu().ay(), entry.imu().az(),
          entry.imu().gx(),       entry.imu().gy(), entry.imu().gz(),
          entry.imu().timestamp()};
}
msensor::Scan3D
SlamPlayer::fromEntryScan3D(const sensors::RecordingEntry &entry) {
  msensor::Scan3D scan;
  for (const auto &pt : entry.scan().points()) {
    scan.points.emplace_back(pt.x(), pt.y(), pt.z());
  }
  scan.timestamp = entry.scan().timestamp();
  return scan;
}

} // namespace mslam