#include "slam/RecordingSensorPlayer.hh"
#include "conversions.hh"
#include "timing/timing.hh"
#include <thread>

namespace mslam {

RecordingSensorPlayer::RecordingSensorPlayer(
    const std::filesystem::path &file, const std::shared_ptr<ILog> &logger,
    bool with_imu, bool with_lidar, unsigned int entry_delay_ms)
    : player_(file), logger_(logger), with_imu_(with_imu),
      with_lidar_(with_lidar), entry_delay_ms_(entry_delay_ms) {}

void RecordingSensorPlayer::init() {}

void RecordingSensorPlayer::startSampling() { started_ = true; }

void RecordingSensorPlayer::stopSampling() {
  started_ = false;
  end_of_file_ = true;
}

std::shared_ptr<msensor::Scan3DI> RecordingSensorPlayer::getScan() {
  if (!started_) {
    return nullptr;
  }

  if (scan_queue_.empty() && !fillUntilScanAvailable()) {
    return nullptr;
  }

  auto scan = scan_queue_.front();
  scan_queue_.pop();
  return scan;
}

std::optional<msensor::IMUData> RecordingSensorPlayer::getImuData() {
  if (!started_ || imu_queue_.empty()) {
    return std::nullopt;
  }

  const auto imu = imu_queue_.front();
  imu_queue_.pop();
  return imu;
}

bool RecordingSensorPlayer::isFinished() const {
  return end_of_file_ && scan_queue_.empty();
}

bool RecordingSensorPlayer::fillUntilScanAvailable() {
  while (!end_of_file_ && scan_queue_.empty()) {
    const auto time_start = timing::getNowUs();

    if (!player_.next()) {
      end_of_file_ = true;
      break;
    }

    const auto &entry = player_.getLastEntry();

    if (entry.entry_case() == sensors::RecordingEntry::kImu) {
      if (with_imu_) {
        imu_queue_.push(fromEntryToImu(entry));
      }
    } else if (entry.entry_case() == sensors::RecordingEntry::kScan) {
      if (with_lidar_) {
        scan_queue_.push(fromEntryScan3D(entry));
      }
    }

    const auto delta_time = timing::getNowUs() - time_start;
    const auto time_delay = entry_delay_ms_ * 1000 - delta_time;

    if (time_delay > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(time_delay));
    }
  }

  if (end_of_file_ && scan_queue_.empty()) {
    logger_->log(ILog::Level::INFO, "Finished sensor recording playback.");
  }

  return !scan_queue_.empty();
}

msensor::IMUData
RecordingSensorPlayer::fromEntryToImu(const sensors::RecordingEntry &entry) {
  return {entry.imu().ax(),       entry.imu().ay(), entry.imu().az(),
          entry.imu().gx(),       entry.imu().gy(), entry.imu().gz(),
          entry.imu().timestamp()};
}

std::shared_ptr<msensor::Scan3DI>
RecordingSensorPlayer::fromEntryScan3D(const sensors::RecordingEntry &entry) {
  return fromProtobuf(entry.scan());
}

} // namespace mslam