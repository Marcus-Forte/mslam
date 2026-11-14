#include "mslam/slam/ImuScanSync.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <memory>

namespace {

constexpr std::uint64_t kMs = 1'000'000ULL;
constexpr unsigned int kPointsPerSecond = 1000; // 1 ms per point
constexpr std::uint64_t kT0 = 5'000 * kMs;      // scan start
constexpr std::size_t kPoints = 101;            // 100 ms: t_end = kT0 + 100 ms
constexpr std::uint64_t kTEnd = kT0 + 100 * kMs;

std::shared_ptr<spdlog::logger> silentLogger() {
  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);
  return logger;
}

mslam::Scan makeScan(std::uint64_t t0 = kT0, std::size_t points = kPoints) {
  mslam::Scan scan;
  scan.header.timestamp = t0;
  scan.points.resize(points);
  return scan;
}

msensor::IMUData makeImu(std::uint64_t timestamp_ns, float ax = 0.0F) {
  msensor::IMUData imu{};
  imu.header.timestamp = timestamp_ns;
  imu.ax = ax;
  return imu;
}

mslam::ImuScanSynchronizer makeSync(bool with_imu = true,
                                    unsigned int pps = kPointsPerSecond) {
  return mslam::ImuScanSynchronizer(with_imu, pps, silentLogger());
}

TEST(ImuScanSync, ScanEndIsTheLastPointTime) {
  EXPECT_EQ(mslam::ImuScanSynchronizer::scanEndTimestamp(makeScan(),
                                                         kPointsPerSecond),
            kTEnd);
  // A single point, and an empty scan, end where they start.
  EXPECT_EQ(mslam::ImuScanSynchronizer::scanEndTimestamp(makeScan(kT0, 1),
                                                         kPointsPerSecond),
            kT0);
  EXPECT_EQ(mslam::ImuScanSynchronizer::scanEndTimestamp(makeScan(kT0, 0),
                                                         kPointsPerSecond),
            kT0);
  // No per-point timing, no scan end.
  EXPECT_FALSE(
      mslam::ImuScanSynchronizer::scanEndTimestamp(makeScan(), 0).has_value());
}

TEST(ImuScanSync, ScanArrivingBeforeItsImuSamplesIsNotCovered) {
  auto sync = makeSync();
  const auto scan = makeScan();
  EXPECT_FALSE(sync.covers(scan)); // nothing yet

  for (std::uint64_t t = kT0 - 20 * kMs; t < kTEnd; t += 5 * kMs) {
    sync.push(makeImu(t));
  }
  EXPECT_FALSE(sync.covers(scan)); // newest sample is 5 ms short of t_end

  sync.push(makeImu(kTEnd + 3 * kMs));
  EXPECT_TRUE(sync.covers(scan));
}

TEST(ImuScanSync, SampleExactlyAtTheScanEndCoversIt) {
  auto sync = makeSync();
  sync.push(makeImu(kT0));
  sync.push(makeImu(kTEnd));
  const auto scan = makeScan();
  ASSERT_TRUE(sync.covers(scan));

  mslam::ImuScanSynchronizer::Window window;
  sync.take(scan, window);
  ASSERT_EQ(window.samples.size(), 2U);
  EXPECT_EQ(window.samples.back().header.timestamp, kTEnd);
  EXPECT_FALSE(window.split.has_value()); // lands on t_end by itself
  EXPECT_EQ(sync.queued(), 0U);
}

TEST(ImuScanSync, SamplesAfterTheScanEndWaitForTheNextScan) {
  auto sync = makeSync();
  // 5 ms samples, offset by 2 ms from the scan end.
  for (std::uint64_t t = kT0 - 3 * kMs; t <= kTEnd + 40 * kMs; t += 5 * kMs) {
    sync.push(makeImu(t));
  }

  mslam::ImuScanSynchronizer::Window window;
  sync.take(makeScan(), window);
  ASSERT_FALSE(window.samples.empty());
  EXPECT_LE(window.samples.back().header.timestamp, kTEnd);
  EXPECT_EQ(window.samples.front().header.timestamp, kT0 - 3 * kMs);
  // The queue keeps only samples later than t_end.
  EXPECT_GT(sync.queued(), 0U);

  // The next scan (the following 100 ms) gets the rest, none twice.
  const auto first_last = window.samples.back().header.timestamp;
  sync.push(makeImu(kTEnd + 200 * kMs));
  mslam::ImuScanSynchronizer::Window next;
  sync.take(makeScan(kTEnd), next);
  ASSERT_FALSE(next.samples.empty());
  EXPECT_GT(next.samples.front().header.timestamp, first_last);
}

TEST(ImuScanSync, StraddlingIntervalIsSplitAtTheScanEnd) {
  auto sync = makeSync();
  sync.push(makeImu(kT0 + 90 * kMs, 1.0F));
  sync.push(makeImu(kTEnd - 2 * kMs, 2.0F)); // last sample before t_end
  sync.push(makeImu(kTEnd + 3 * kMs, 3.0F)); // first one after

  mslam::ImuScanSynchronizer::Window window;
  sync.take(makeScan(), window);

  ASSERT_EQ(window.samples.size(), 2U);
  ASSERT_TRUE(window.split.has_value());
  // The split is the sample that ends the straddling interval, moved to t_end:
  // [t_end - 2 ms, t_end] now, [t_end, t_end + 3 ms] when the real one arrives.
  EXPECT_EQ(window.split->header.timestamp, kTEnd);
  EXPECT_EQ(window.split->ax, 3.0F);
  EXPECT_EQ(sync.queued(), 1U); // the real sample stays for the next scan
  EXPECT_EQ(sync.stats().splits, 1U);
}

TEST(ImuScanSync, NoSplitWithoutAnEarlierSampleOrAcrossAnOutage) {
  {
    auto sync = makeSync();
    sync.push(makeImu(kTEnd + 3 * kMs)); // the first sample ever, after t_end
    mslam::ImuScanSynchronizer::Window window;
    sync.take(makeScan(), window);
    EXPECT_TRUE(window.samples.empty());
    EXPECT_FALSE(window.split.has_value());
  }
  {
    // An IMU outage: the next sample comes 5 s after the last one. The scans
    // inside it get no IMU data instead of that sample held backwards.
    auto sync = makeSync();
    sync.push(makeImu(kT0 - 10 * kMs));
    sync.push(makeImu(kTEnd + 5'000 * kMs));
    mslam::ImuScanSynchronizer::Window window;
    sync.take(makeScan(), window);
    EXPECT_EQ(window.samples.size(), 1U);
    EXPECT_FALSE(window.split.has_value());
    EXPECT_EQ(sync.queued(), 1U);
  }
}

TEST(ImuScanSync, UncoveredScanGetsWhatHasArrivedAndIsCounted) {
  auto sync = makeSync();
  sync.push(makeImu(kT0));
  sync.push(makeImu(kT0 + 5 * kMs));
  const auto scan = makeScan();
  ASSERT_FALSE(sync.covers(scan));

  mslam::ImuScanSynchronizer::Window window;
  sync.take(scan, window);
  EXPECT_EQ(window.samples.size(), 2U);
  EXPECT_FALSE(window.split.has_value());
  EXPECT_EQ(sync.stats().scans, 1U);
  EXPECT_EQ(sync.stats().scans_uncovered, 1U);
}

TEST(ImuScanSync, DisabledImuIgnoresSamplesAndCoversEveryScan) {
  auto sync = makeSync(/*with_imu=*/false);
  sync.push(makeImu(kT0));
  sync.push(makeImu(kT0 + 5 * kMs));
  EXPECT_EQ(sync.queued(), 0U);
  EXPECT_TRUE(sync.covers(makeScan()));

  mslam::ImuScanSynchronizer::Window window;
  window.samples.push_back(makeImu(1)); // leftovers are cleared
  sync.take(makeScan(), window);
  EXPECT_TRUE(window.samples.empty());
  EXPECT_FALSE(window.split.has_value());
}

TEST(ImuScanSync, WithoutPerPointTimingAScanTakesEverythingQueued) {
  auto sync = makeSync(/*with_imu=*/true, /*pps=*/0);
  const auto scan = makeScan();
  EXPECT_TRUE(sync.covers(scan)); // nothing to wait for, even with no samples

  for (std::uint64_t t = kT0; t < kT0 + 500 * kMs; t += 5 * kMs) {
    sync.push(makeImu(t));
  }
  mslam::ImuScanSynchronizer::Window window;
  sync.take(scan, window);
  EXPECT_EQ(window.samples.size(), 100U);
  EXPECT_FALSE(window.split.has_value());
  EXPECT_EQ(sync.queued(), 0U);
}

TEST(ImuScanSync, DropsStaleSamplesAndRestartsOnAClockJump) {
  auto sync = makeSync();
  const std::uint64_t base = kT0 + 20'000 * kMs;
  sync.push(makeImu(base + 50 * kMs));
  sync.push(makeImu(base + 40 * kMs)); // late: older than the newest
  EXPECT_EQ(sync.queued(), 1U);
  EXPECT_EQ(sync.stats().stale_dropped, 1U);

  // The clock restarts 20 s earlier: not a late sample, the queue is restarted.
  sync.push(makeImu(kT0));
  EXPECT_EQ(sync.queued(), 1U);
  EXPECT_EQ(sync.stats().clock_jumps, 1U);
  EXPECT_EQ(sync.stats().stale_dropped, 1U);
}

TEST(ImuScanSync, QueueIsBoundedAndClearForgetsEverything) {
  auto sync = makeSync();
  const std::size_t extra = 10;
  for (std::size_t i = 0;
       i < mslam::ImuScanSynchronizer::kMaxQueuedSamples + extra; ++i) {
    sync.push(makeImu(kT0 + i * kMs));
  }
  EXPECT_EQ(sync.queued(), mslam::ImuScanSynchronizer::kMaxQueuedSamples);
  EXPECT_EQ(sync.stats().overflow_dropped, extra);

  sync.clear();
  EXPECT_EQ(sync.queued(), 0U);
  EXPECT_FALSE(sync.covers(makeScan()));
}

} // namespace
