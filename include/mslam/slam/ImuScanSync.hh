#pragma once

#include "mslam/common/Points.hh"

#include "msensor/interface/IImu.hh"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <spdlog/logger.h>
#include <vector>

namespace mslam {

/// Counters of what the synchroniser did.
struct ImuSyncStats {
  std::size_t scans = 0;           ///< scans given their IMU window
  std::size_t scans_uncovered = 0; ///< ... released before the IMU reached them
  std::size_t splits = 0;          ///< scan ends that fell between two samples
  std::size_t stale_dropped = 0;   ///< samples older than one already queued
  std::size_t overflow_dropped = 0; ///< oldest samples dropped, queue full
  std::size_t clock_jumps = 0;      ///< IMU clock went back by more than 1 s
};

/**
 * @brief Groups IMU samples with scans by timestamp, not by arrival.
 *
 * LiDAR and IMU reach mslam on separate streams, so the samples queued when a
 * scan arrives are not the samples of that scan. A scan covers
 * `[t0, t_end]`, `t0 = scan.header.timestamp` (first point) and
 * `t_end = t0 + (N - 1) / points_per_second` (last point, the same rule the
 * deskew uses). take() hands out exactly the queued samples with
 * `timestamp <= t_end`; later samples wait for the next scan.
 *
 * The estimator should sit exactly on `t_end` when the scan is registered, so
 * when `t_end` falls between two samples the straddling interval is split in
 * two: take() also returns a copy of the first later sample, stamped `t_end`.
 * Feeding it to the estimator first integrates `[last sample, t_end]` (the
 * sample's value is held backwards over its interval, as the estimator does),
 * and feeding the real sample later integrates the remaining
 * `[t_end, sample]`. Nothing is lost or counted twice.
 *
 * Without per-point timing (`points_per_second == 0`) there is no `t_end`: a
 * scan is always covered and takes everything queued (the arrival grouping).
 * With the IMU disabled nothing is queued and every scan is covered.
 *
 * The queue is bounded (kMaxQueuedSamples, oldest dropped). Not thread-safe.
 */
class ImuScanSynchronizer {
public:
  /// Queue bound: 5 s of a 200 Hz IMU.
  static constexpr std::size_t kMaxQueuedSamples = 1000;

  /// The samples of one scan, see take().
  struct Window {
    std::vector<msensor::IMUData> samples; ///< timestamp <= t_end, in order
    /// The straddling sample re-stamped to `t_end`, if `t_end` fell between
    /// two samples. Feed it after #samples, before the scan.
    std::optional<msensor::IMUData> split;
  };

  /// @param with_imu When false, push() ignores samples and every scan is
  ///        covered and gets an empty window.
  ImuScanSynchronizer(bool with_imu, unsigned int points_per_second,
                      std::shared_ptr<spdlog::logger> logger);

  /// `t_end` of @p scan in nanoseconds, or nullopt without per-point timing.
  static std::optional<std::uint64_t>
  scanEndTimestamp(const Scan &scan, unsigned int points_per_second);

  /// Queues a sample. Samples older than the newest one seen are dropped.
  void push(const msensor::IMUData &imu);

  /// True if the IMU has reached the end of @p scan (a sample with
  /// `timestamp >= t_end` arrived), so take() gives its complete window.
  bool covers(const Scan &scan) const;

  /// Removes the samples of @p scan from the queue into @p window (replaced,
  /// capacity reused). Works for an uncovered scan too: it then gets what has
  /// arrived and counts as `scans_uncovered`.
  void take(const Scan &scan, Window &window);

  /// Forgets the queued samples and the sample clock.
  void clear();

  std::size_t queued() const { return queue_.size(); }
  const ImuSyncStats &stats() const { return stats_; }

private:
  bool with_imu_;
  unsigned int points_per_second_;
  std::shared_ptr<spdlog::logger> logger_;
  std::deque<msensor::IMUData> queue_;
  // Newest timestamp seen, and the one the estimator's IMU clock is at.
  std::optional<std::uint64_t> newest_ns_;
  std::optional<std::uint64_t> taken_ns_;
  ImuSyncStats stats_;
};

} // namespace mslam
