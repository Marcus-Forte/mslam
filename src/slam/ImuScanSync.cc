#include "mslam/slam/ImuScanSync.hh"

#include <algorithm>
#include <cmath>

namespace mslam {

namespace {
constexpr double kNanosecondsPerSecond = 1e9;
// A backwards jump of the IMU clock bigger than this is a restart of the
// clock (a replay looping, a sensor reset), not a late sample.
constexpr std::uint64_t kClockJumpNs = 1'000'000'000ULL;
// The estimator treats an IMU interval longer than this as a timestamp problem
// and resets (Slam::Predict). Such an interval is not split either: the
// scans inside an IMU outage get no IMU data, rather than the value of the
// sample that ended the outage held backwards over seconds.
constexpr std::uint64_t kMaxSplitIntervalNs = 1'000'000'000ULL;
} // namespace

ImuScanSynchronizer::ImuScanSynchronizer(bool with_imu,
                                         unsigned int points_per_second,
                                         std::shared_ptr<spdlog::logger> logger)
    : with_imu_(with_imu), points_per_second_(points_per_second),
      logger_(std::move(logger)) {}

std::optional<std::uint64_t>
ImuScanSynchronizer::scanEndTimestamp(const Scan &scan,
                                      unsigned int points_per_second) {
  if (points_per_second == 0) {
    return std::nullopt;
  }
  const double span_ns = scan.points.empty()
                             ? 0.0
                             : static_cast<double>(scan.points.size() - 1) *
                                   kNanosecondsPerSecond /
                                   static_cast<double>(points_per_second);
  return scan.header.timestamp +
         static_cast<std::uint64_t>(std::llround(span_ns));
}

void ImuScanSynchronizer::push(const msensor::IMUData &imu) {
  if (!with_imu_) {
    return;
  }
  const std::uint64_t timestamp = imu.header.timestamp;
  if (newest_ns_.has_value() && timestamp < *newest_ns_) {
    if (*newest_ns_ - timestamp > kClockJumpNs) {
      ++stats_.clock_jumps;
      logger_->warn("IMU clock went back by {:.3f} s: restarting the IMU "
                    "queue",
                    static_cast<double>(*newest_ns_ - timestamp) * 1e-9);
      clear();
    } else {
      ++stats_.stale_dropped;
      logger_->warn("Dropping IMU sample {} ns older than the newest one",
                    *newest_ns_ - timestamp);
      return;
    }
  }
  newest_ns_ = timestamp;
  queue_.push_back(imu);
  if (queue_.size() > kMaxQueuedSamples) {
    queue_.pop_front();
    if (++stats_.overflow_dropped == 1) {
      logger_->warn("IMU queue full ({} samples): dropping the oldest. Are "
                    "scans arriving?",
                    kMaxQueuedSamples);
    }
  }
}

bool ImuScanSynchronizer::covers(const Scan &scan) const {
  if (!with_imu_) {
    return true;
  }
  const auto end = scanEndTimestamp(scan, points_per_second_);
  if (!end.has_value()) {
    return true;
  }
  return newest_ns_.has_value() && *newest_ns_ >= *end;
}

void ImuScanSynchronizer::take(const Scan &scan, Window &window) {
  window.samples.clear();
  window.split.reset();
  if (!with_imu_) {
    return;
  }
  ++stats_.scans;
  if (!covers(scan)) {
    ++stats_.scans_uncovered;
  }

  const auto end = scanEndTimestamp(scan, points_per_second_);
  while (!queue_.empty() &&
         (!end.has_value() || queue_.front().header.timestamp <= *end)) {
    taken_ns_ = queue_.front().header.timestamp;
    window.samples.push_back(queue_.front());
    queue_.pop_front();
  }

  // The scan end fell inside the interval ending at the next sample: hand out
  // the first part of it. Without an earlier sample there is no interval yet.
  if (end.has_value() && !queue_.empty() && taken_ns_.has_value() &&
      *taken_ns_ < *end &&
      queue_.front().header.timestamp - *taken_ns_ <= kMaxSplitIntervalNs) {
    window.split = queue_.front();
    window.split->header.timestamp = *end;
    taken_ns_ = *end;
    ++stats_.splits;
  }
}

void ImuScanSynchronizer::clear() {
  queue_.clear();
  newest_ns_.reset();
  taken_ns_.reset();
}

} // namespace mslam
