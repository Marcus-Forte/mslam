#pragma once

#include "ILog.hh"
#include "config/IConfig.hh"
#include "imu/IImu.hh"
#include "map/IMap.hh"
#include "recorder/ScanPlayer.hh"
#include <filesystem>

namespace mslam {

/**
 * @brief Loads a file with scans and runs the slam over the recorded
 * measurements
 *
 */

class SlamPlayer {
public:
  SlamPlayer(const std::filesystem::path &file,
             const std::shared_ptr<ILog> &logger,
             const SlamConfiguration &config, const std::shared_ptr<IMap> &map);

  /**
   * @brief Runs the SLAM with the scans from the file.
   * Blocking call until file is fully processed.
   */
  void run();

private:
  msensor::ScanPlayer player_;
  std::shared_ptr<IMap> map_;
  std::shared_ptr<ILog> logger_;
  SlamConfiguration config_;

  msensor::IMUData fromEntryToImu(const sensors::RecordingEntry &entry);
  msensor::Scan3D fromEntryScan3D(const sensors::RecordingEntry &entry);
};

} // namespace mslam