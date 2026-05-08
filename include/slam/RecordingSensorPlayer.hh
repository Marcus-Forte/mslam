#pragma once

#include "ILog.hh"
#include "config/IConfig.hh"
#include "interface/IImu.hh"
#include "interface/ILidar.hh"
#include "recorder/scan_player.hh"
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
                        const std::shared_ptr<ILog> &logger,
                        const SlamConfiguration &config);

  void init() override;
  void startSampling() override;
  void stopSampling() override;
  std::shared_ptr<msensor::Scan3DI> getScan() override;
  std::optional<msensor::IMUData> getImuData() override;

private:
  msensor::ScanPlayer player_;
  std::shared_ptr<ILog> logger_;
  SlamConfiguration config_;
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