#pragma once

#include "ILog.hh"
#include "config/IConfig.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"
#include "msensor/recorder/scan_player.hh"
#include <filesystem>
#include <queue>

namespace mslam {

/**
 * @brief Loads a recorded sensor file and exposes it through the ILidar / IImu
 *        interfaces.
 */
class RecordingSensorPlayer : public msensor::ILidar, public msensor::IImu {
public:
  RecordingSensorPlayer(const std::filesystem::path &file,
                        const std::shared_ptr<ILog> &logger, bool with_imu,
                        bool with_lidar, unsigned int entry_delay_ms);

  void init() override;
  void startSampling() override;
  void stopSampling() override;
  std::shared_ptr<msensor::Scan3DI> getScan() override;
  std::optional<msensor::IMUData> getImuData() override;
  bool isFinished() const;

private:
  msensor::ScanPlayer player_;
  std::shared_ptr<ILog> logger_;
  bool with_imu_ = false;
  bool with_lidar_ = true;
  unsigned int entry_delay_ms_ = 0;
  bool started_ = false;
  bool end_of_file_ = false;
  std::queue<std::shared_ptr<msensor::Scan3DI>> scan_queue_;
  std::queue<msensor::IMUData> imu_queue_;

  bool fillUntilScanAvailable();

  msensor::IMUData fromEntryToImu(const sensors::RecordingEntry &entry);
  std::shared_ptr<msensor::Scan3DI>
  fromEntryScan3D(const sensors::RecordingEntry &entry);
};

} // namespace mslam