#include "SyntheticScene.hh"

#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/SensorInput.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/SlamPipeline.hh"
#include "mslam/slam/SlamRunner.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

std::shared_ptr<spdlog::logger> silentLogger() {
  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);
  return logger;
}

mslam::SlamConfiguration testConfig() {
  mslam::SlamConfiguration config;
  config.with_imu = false;
  config.with_lidar = true;
  config.preprocessor.voxel_size = 0.1F;
  config.preprocessor.min_distance_to_center = 0.5F;
  config.preprocessor.downsample_filter = mslam::DownsampleFilter::VoxelHash;
  config.map_parameters.resolution = 0.1F;
  config.map_parameters.max_points_per_voxel = 5;
  config.parameters.reg_iterations = 3;
  config.parameters.opt_iterations = 3;
  config.parameters.max_correspondence_distance = 0.3F;
  return config;
}

// Records what the pipeline publishes. Thread-safe: the runner test reads it
// from the test thread while the SLAM thread writes.
class RecordingOutput final : public mslam::ISlamOutput {
public:
  void updatePose(const mslam::Pose3D &pose) override {
    std::lock_guard lock(mutex_);
    last_pose_ = pose;
    ++pose_updates_;
  }
  void updateTransformedScan(mslam::PointCloud scan) override {
    std::lock_guard lock(mutex_);
    last_scan_size_ = scan.size();
    ++scans_;
  }
  void updateMapIncrement(mslam::PointCloud increment) override {
    std::lock_guard lock(mutex_);
    last_increment_size_ = increment.size();
    ++increments_;
  }
  void reset() override {
    std::lock_guard lock(mutex_);
    ++resets_;
  }

  /// Polls until @p predicate holds (called with the lock held), up to 10 s.
  /// (Polling rather than a condition variable: the thread sanitizer runtime
  /// of the dev container's toolchain crashes in condition_variable waits.)
  template <class Predicate> bool waitFor(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard lock(mutex_);
        if (predicate(*this)) {
          return true;
        }
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  std::mutex mutex_;
  mslam::Pose3D last_pose_ = mslam::Pose3D::Zero();
  std::size_t last_scan_size_ = 0;
  std::size_t last_increment_size_ = 0;
  int pose_updates_ = 0;
  int scans_ = 0;
  int increments_ = 0;
  int resets_ = 0;
};

struct Fixture {
  explicit Fixture(mslam::SlamConfiguration config = testConfig())
      : config(config), map(std::make_shared<mslam::VoxelHashMap>(
                            config.map_parameters.resolution,
                            config.map_parameters.max_points_per_voxel)),
        slam(silentLogger(), config, map),
        pipeline(slam, config, silentLogger(), output) {
    map->setNumAdjacentVoxelSearch(1);
  }

  mslam::SlamConfiguration config;
  std::shared_ptr<mslam::VoxelHashMap> map;
  mslam::Slam slam;
  RecordingOutput output;
  mslam::SlamPipeline pipeline;
};

mslam::Scan roomScan(std::uint64_t sequence = 0) {
  mslam::Scan scan;
  scan.header.timestamp = 1'000'000'000ULL + sequence * 100'000'000ULL;
  scan.header.sequence_number = sequence;
  scan.points = test::sampleRoom(400.0F, 1);
  return scan;
}

TEST(SlamPipeline, FirstScansSeedTheMapAndAreConsumedWithoutRegistration) {
  Fixture f;
  const auto scan = roomScan();
  for (int i = 0; i < mslam::SlamPipeline::kInitScans; ++i) {
    f.pipeline.processScan(scan, {});
  }
  EXPECT_EQ(f.output.scans_, mslam::SlamPipeline::kInitScans);
  EXPECT_EQ(f.output.increments_, mslam::SlamPipeline::kInitScans);
  // The later init scans add nothing new: voxels are already full.
  EXPECT_GT(f.map->size(), 0U);
  EXPECT_EQ(f.pipeline.timings().update.count(), 0);
  EXPECT_EQ(f.pipeline.timings().scans,
            static_cast<std::size_t>(mslam::SlamPipeline::kInitScans));
}

TEST(SlamPipeline, ScanAfterInitIsRegisteredAndStaysAtTheOrigin) {
  Fixture f;
  const auto scan = roomScan();
  for (int i = 0; i < mslam::SlamPipeline::kInitScans + 3; ++i) {
    f.pipeline.processScan(scan, {});
  }
  EXPECT_EQ(f.output.scans_, mslam::SlamPipeline::kInitScans + 3);
  EXPECT_GT(f.pipeline.timings().update.count(), 0);
  EXPECT_GT(f.pipeline.timings().preprocess.count(), 0);
  // Identical scans register to the identity.
  EXPECT_NEAR(f.output.last_pose_.head<3>().norm(), 0.0, 5e-3);
  EXPECT_NEAR(f.output.last_pose_.tail<3>().norm(), 0.0, 5e-3);
  EXPECT_GT(f.output.last_scan_size_, 100U);
}

TEST(SlamPipeline, ImuSamplesArePredictedAndPublishThePose) {
  mslam::SlamConfiguration config = testConfig();
  config.with_imu = true;
  config.imu_acceleration_scale = 1.0;
  Fixture f(config);

  msensor::IMUData imu{};
  imu.az = 9.80665F;
  imu.header.timestamp = 1'000'000'000ULL;
  f.pipeline.processImu(imu);
  imu.header.timestamp = 1'005'000'000ULL;
  f.pipeline.processImu(imu);
  EXPECT_EQ(f.output.pose_updates_, 2);
  EXPECT_GE(f.pipeline.timings().predict.count(), 0);
}

TEST(SlamPipeline, ResetClearsMapPoseAndNotifiesTheOutput) {
  Fixture f;
  const auto scan = roomScan();
  for (int i = 0; i < mslam::SlamPipeline::kInitScans + 2; ++i) {
    f.pipeline.processScan(scan, {});
  }
  ASSERT_GT(f.map->size(), 0U);

  f.pipeline.reset();

  EXPECT_EQ(f.map->size(), 0U);
  EXPECT_EQ(f.output.resets_, 1);
  EXPECT_EQ(f.output.last_pose_, mslam::Pose3D::Zero());
  EXPECT_EQ(f.slam.getPose(), mslam::Pose3D::Zero());
}

TEST(SlamPipeline, LidarDisabledOnlyConsumesInitScans) {
  mslam::SlamConfiguration config = testConfig();
  config.with_lidar = false;
  Fixture f(config);
  const auto scan = roomScan();
  for (int i = 0; i < mslam::SlamPipeline::kInitScans + 3; ++i) {
    f.pipeline.processScan(scan, {});
  }
  EXPECT_EQ(f.output.scans_, mslam::SlamPipeline::kInitScans);
}

// ---------------------------------------------------------------------------
// IMU / scan synchronisation

constexpr std::uint64_t kScanPeriodNs = 100'000'000ULL;
constexpr std::uint64_t kImuPeriodNs = 5'000'000ULL;
constexpr std::uint64_t kScanBaseNs = 1'000'000'000ULL;

/// A room scan lasting exactly one scan period at the configured point rate.
mslam::Scan timedRoomScan(int index) {
  mslam::Scan scan = roomScan();
  scan.header.timestamp =
      kScanBaseNs + static_cast<std::uint64_t>(index) * kScanPeriodNs;
  scan.header.sequence_number = static_cast<std::uint64_t>(index);
  return scan;
}

mslam::SlamConfiguration imuConfig() {
  mslam::SlamConfiguration config = testConfig();
  config.with_imu = true;
  config.imu_acceleration_scale = 1.0;
  // The room scan spans exactly 100 ms at this rate.
  const auto points = roomScan().points.size();
  config.preprocessor.points_per_second =
      static_cast<unsigned int>((points - 1) * 10);
  return config;
}

msensor::IMUData stationaryImu(std::uint64_t timestamp_ns) {
  msensor::IMUData imu{};
  imu.az = 9.80665F;
  imu.header.timestamp = timestamp_ns;
  return imu;
}

// IMU samples every 5 ms, 2 ms off the scan boundaries so every scan end
// falls inside an interval, and delivered a whole scan late (as when the IMU
// stream lags the LiDAR): each scan must still be given the samples of its own
// window, and every interval of the IMU time must be integrated exactly once.
TEST(SlamPipeline, EachScanIntegratesExactlyTheImuTimeOfItsWindow) {
  Fixture f(imuConfig());
  constexpr int kScans = mslam::SlamPipeline::kInitScans + 12;
  const std::uint64_t imu_offset = 2'000'000ULL;

  std::uint64_t next_imu = kScanBaseNs - 50'000'000ULL + imu_offset;
  std::vector<double> integrated; // after each real scan
  for (int i = 0; i < kScans; ++i) {
    const auto scan = timedRoomScan(i);
    // The IMU stream is ahead of the scan by the time the scan is processed
    // (the scan waits for coverage), but never delivers beyond it by much.
    const std::uint64_t t_end = scan.header.timestamp + kScanPeriodNs;
    while (next_imu <= t_end + 3 * kImuPeriodNs) {
      f.pipeline.pushImu(stationaryImu(next_imu));
      next_imu += kImuPeriodNs;
    }
    ASSERT_TRUE(f.pipeline.imuCovers(scan)) << i;
    f.pipeline.processSyncedScan(scan);
    if (i >= mslam::SlamPipeline::kInitScans) {
      integrated.push_back(f.slam.registrationStats().preintegrated_seconds);
    }
  }

  const auto &stats = f.slam.registrationStats();
  EXPECT_EQ(stats.lidar_only_scans, 0U);
  EXPECT_EQ(stats.imu_scans, 12U);
  // After the first real scan (the init scans reset the IMU clock on
  // purpose) every scan integrates exactly one scan period: nothing lost at the
  // scan boundary, nothing from the next window.
  for (std::size_t i = 1; i < integrated.size(); ++i) {
    EXPECT_NEAR(integrated[i] - integrated[i - 1], 0.1, 1e-9) << i;
  }
  EXPECT_EQ(f.pipeline.syncStats().scans_uncovered, 0U);
  EXPECT_EQ(f.pipeline.syncStats().splits, static_cast<std::size_t>(kScans));
}

// The same IMU samples with the scans out of step used to be grouped by
// arrival; by timestamp, a scan whose samples have not all arrived is held.
TEST(SlamPipeline, ScanIsNotCoveredUntilTheImuReachesItsEnd) {
  Fixture f(imuConfig());
  const auto scan = timedRoomScan(0);
  EXPECT_FALSE(f.pipeline.imuCovers(scan));
  for (std::uint64_t t = kScanBaseNs; t < kScanBaseNs + kScanPeriodNs;
       t += kImuPeriodNs) {
    f.pipeline.pushImu(stationaryImu(t));
  }
  EXPECT_FALSE(f.pipeline.imuCovers(scan)); // last sample is 5 ms short
  f.pipeline.pushImu(stationaryImu(kScanBaseNs + kScanPeriodNs));
  EXPECT_TRUE(f.pipeline.imuCovers(scan));
}

TEST(SlamPipeline, WithoutImuEveryScanIsCoveredAndRegistersLidarOnly) {
  Fixture f;                                      // with_imu = false
  f.pipeline.pushImu(stationaryImu(kScanBaseNs)); // ignored
  const auto scan = timedRoomScan(0);
  EXPECT_TRUE(f.pipeline.imuCovers(scan));
  for (int i = 0; i < mslam::SlamPipeline::kInitScans + 2; ++i) {
    f.pipeline.processSyncedScan(timedRoomScan(i));
  }
  EXPECT_EQ(f.slam.registrationStats().imu_scans, 0U);
  EXPECT_EQ(f.slam.registrationStats().lidar_only_scans, 2U);
  EXPECT_EQ(f.pipeline.syncStats().scans, 0U);
}

TEST(SlamPipeline, WithoutPerPointTimingScansAreNeverHeld) {
  mslam::SlamConfiguration config = imuConfig();
  config.preprocessor.points_per_second = 0;
  Fixture f(config);
  const auto scan = timedRoomScan(0);
  EXPECT_TRUE(f.pipeline.imuCovers(scan)); // no scan end to wait for
  // Everything queued goes with the scan (the arrival grouping).
  for (int i = 0; i < 20; ++i) {
    f.pipeline.pushImu(stationaryImu(kScanBaseNs + i * kImuPeriodNs));
  }
  f.pipeline.processSyncedScan(scan);
  EXPECT_EQ(f.pipeline.syncStats().splits, 0U);
  EXPECT_EQ(f.pipeline.syncStats().scans, 1U);
}

// ---------------------------------------------------------------------------
// SlamRunner

class FakeSensors final : public msensor::ILidar, public msensor::IImu {
public:
  void init() override {}
  void startSampling() override {}
  void stopSampling() override {}
  void setScanCallback(ScanCallback callback) override {
    std::lock_guard lock(mutex_);
    scan_callback_ = std::move(callback);
  }
  void setImuCallback(ImuCallback callback) override {
    std::lock_guard lock(mutex_);
    imu_callback_ = std::move(callback);
  }

  bool hasScanCallback() {
    std::lock_guard lock(mutex_);
    return static_cast<bool>(scan_callback_);
  }

  void pushScan(mslam::Scan scan) {
    std::lock_guard lock(mutex_);
    if (scan_callback_) {
      scan_callback_(std::move(scan));
    }
  }

  void pushImu(const msensor::IMUData &imu) {
    std::lock_guard lock(mutex_);
    if (imu_callback_) {
      imu_callback_(imu);
    }
  }

private:
  std::mutex mutex_;
  ScanCallback scan_callback_;
  ImuCallback imu_callback_;
};

class RunnerFixture : public ::testing::Test {
protected:
  RunnerFixture() = default;
  explicit RunnerFixture(mslam::SlamConfiguration config)
      : f(std::move(config)) {}

  void SetUp() override {
    sensors = std::make_shared<FakeSensors>();
    runner = std::make_unique<mslam::SlamRunner>(f.pipeline, sensors, sensors,
                                                 f.config, silentLogger());
    thread = std::thread([this] { runner->run(stop); });
    // run() registers its callbacks first thing.
    while (!sensors->hasScanCallback()) {
      std::this_thread::sleep_for(1ms);
    }
  }

  void TearDown() override {
    stop.store(true);
    if (thread.joinable()) {
      thread.join();
    }
  }

  /// The runner keeps only the newest scan, so feed one at a time.
  void feedAndWait(int expected_scans) {
    sensors->pushScan(roomScan());
    ASSERT_TRUE(f.output.waitFor(
        [&](const RecordingOutput &o) { return o.scans_ >= expected_scans; }));
  }

  Fixture f;
  std::shared_ptr<FakeSensors> sensors;
  std::unique_ptr<mslam::SlamRunner> runner;
  std::atomic<bool> stop{false};
  std::thread thread;
};

class ImuRunnerFixture : public RunnerFixture {
protected:
  ImuRunnerFixture() : RunnerFixture(imuConfig()) {}

  void pushImuUpTo(std::uint64_t end_ns) {
    for (; next_imu_ <= end_ns; next_imu_ += kImuPeriodNs) {
      sensors->pushImu(stationaryImu(next_imu_));
    }
  }

  std::uint64_t next_imu_ = kScanBaseNs;
};

// A scan that arrives before its IMU samples is held, not processed with
// whatever happens to be queued; it is released as soon as the IMU reaches its
// end.
TEST_F(ImuRunnerFixture, ScanWaitsForItsImuSamples) {
  const auto scan = timedRoomScan(0);
  sensors->pushScan(scan);
  pushImuUpTo(scan.header.timestamp + kScanPeriodNs - kImuPeriodNs);
  std::this_thread::sleep_for(20ms); // well inside kMaxImuWait
  EXPECT_EQ(f.output.scans_, 0);

  pushImuUpTo(scan.header.timestamp + kScanPeriodNs + kImuPeriodNs);
  ASSERT_TRUE(
      f.output.waitFor([](const RecordingOutput &o) { return o.scans_ == 1; }));
  EXPECT_EQ(runner->imuWaitTimeouts(), 0U);
  EXPECT_EQ(f.pipeline.syncStats().scans_uncovered, 0U);
}

// A stalled IMU stream cannot block the scans forever: after kMaxImuWait the
// scan is processed with what has arrived.
TEST_F(ImuRunnerFixture, BoundedWaitExpiresWhenTheImuStalls) {
  const auto scan = timedRoomScan(0);
  const auto start = std::chrono::steady_clock::now();
  sensors->pushScan(scan);
  ASSERT_TRUE(
      f.output.waitFor([](const RecordingOutput &o) { return o.scans_ == 1; }));
  const auto waited = std::chrono::steady_clock::now() - start;

  EXPECT_GE(waited, mslam::SlamRunner::kMaxImuWait);
  EXPECT_EQ(runner->imuWaitTimeouts(), 1U);
  EXPECT_EQ(f.pipeline.syncStats().scans_uncovered, 1U);
}

// A scan that is waiting does not stop the following ones from being processed
// once its window is complete, and they keep their order.
TEST_F(ImuRunnerFixture, ScansAreProcessedInOrderOnceCovered) {
  pushImuUpTo(kScanBaseNs + 4 * kScanPeriodNs);
  for (int i = 0; i < 3; ++i) {
    sensors->pushScan(timedRoomScan(i));
    ASSERT_TRUE(f.output.waitFor(
        [i](const RecordingOutput &o) { return o.scans_ == i + 1; }));
  }
  EXPECT_EQ(runner->imuWaitTimeouts(), 0U);
  EXPECT_EQ(f.pipeline.syncStats().scans, 3U);
}

TEST_F(RunnerFixture, ProcessesScansAndStopsWhenAsked) {
  for (int i = 1; i <= 3; ++i) {
    feedAndWait(i);
  }
  EXPECT_TRUE(runner->isRunning());
  EXPECT_GT(f.map->size(), 0U);

  stop.store(true);
  thread.join();
  // run() unregistered its callbacks on exit.
  EXPECT_FALSE(sensors->hasScanCallback());
}

TEST_F(RunnerFixture, ResetIsAppliedByTheLoopBetweenScans) {
  feedAndWait(1);
  feedAndWait(2);
  ASSERT_GT(f.map->size(), 0U);

  runner->requestReset();
  ASSERT_TRUE(f.output.waitFor(
      [](const RecordingOutput &o) { return o.resets_ == 1; }));

  // The loop is stopped after a reset, and the state is cleared.
  EXPECT_FALSE(runner->isRunning());
  EXPECT_EQ(f.map->size(), 0U);
  EXPECT_EQ(f.output.last_pose_, mslam::Pose3D::Zero());

  // Scans are ignored while stopped...
  sensors->pushScan(roomScan());
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(f.output.scans_, 2);

  // ...and processing resumes on start.
  runner->startProcessing();
  ASSERT_TRUE(
      f.output.waitFor([](const RecordingOutput &o) { return o.scans_ >= 3; }));
  EXPECT_GT(f.map->size(), 0U);
}

TEST_F(RunnerFixture, StopAndStartControlProcessing) {
  feedAndWait(1);
  runner->stopProcessing();
  EXPECT_FALSE(runner->isRunning());
  sensors->pushScan(roomScan());
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(f.output.scans_, 1);
  runner->startProcessing();
  ASSERT_TRUE(
      f.output.waitFor([](const RecordingOutput &o) { return o.scans_ >= 2; }));
}

// Concurrent reset requests while scans are being processed must neither
// crash nor deadlock; the interesting check is under a thread sanitizer.
TEST_F(RunnerFixture, ResetRequestsRaceFreelyWithScanProcessing) {
  std::atomic<bool> feeding{true};
  std::thread feeder([&] {
    while (feeding.load()) {
      sensors->pushScan(roomScan());
      std::this_thread::sleep_for(1ms);
    }
  });
  for (int i = 0; i < 20; ++i) {
    runner->requestReset();
    std::this_thread::sleep_for(3ms);
    runner->startProcessing();
    std::this_thread::sleep_for(3ms);
  }
  feeding.store(false);
  feeder.join();
  runner->requestReset();
  ASSERT_TRUE(f.output.waitFor(
      [](const RecordingOutput &o) { return o.resets_ >= 1; }));
  // After the last reset is applied the state is empty.
  EXPECT_FALSE(runner->isRunning());
}

// Scans the sensor input had to replace are counted, so the runner can log
// them.
TEST(SensorInput, CountsScansReplacedInTheQueue) {
  auto sensors = std::make_shared<FakeSensors>();
  mslam::SensorInput input(sensors, sensors, true, true);
  EXPECT_EQ(input.droppedScans(), 0U);

  sensors->pushScan(timedRoomScan(0));
  sensors->pushScan(timedRoomScan(1)); // replaces scan 0
  sensors->pushScan(timedRoomScan(2)); // replaces scan 1
  EXPECT_EQ(input.droppedScans(), 2U);

  const auto scan = input.nextScan(); // the newest one is kept
  ASSERT_TRUE(scan);
  EXPECT_EQ(scan->header.sequence_number, 2U);
  EXPECT_EQ(input.nextScan(), nullptr);
}

// With SensorOverflow::Block a full queue holds the sensor's thread back
// instead of replacing a scan: nothing is dropped and every scan comes out in
// order.
TEST(SensorInput, BlockingInputHoldsTheSourceBackAndDropsNothing) {
  auto sensors = std::make_shared<FakeSensors>();
  mslam::SensorInput input(sensors, sensors, true, true,
                           {.overflow = mslam::SensorOverflow::Block});

  constexpr std::uint32_t kScans = 50;
  std::atomic<std::uint32_t> pushed{0};
  std::thread source([&] {
    for (std::uint32_t i = 0; i < kScans; ++i) {
      sensors->pushScan(timedRoomScan(i));
      pushed.store(i + 1);
    }
  });

  // The queue holds one scan, so the source gets one scan in and then waits in
  // its second push: it cannot run ahead of the consumer.
  while (pushed.load() < 1) {
    std::this_thread::sleep_for(1ms);
  }
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(pushed.load(), 1U);

  for (std::uint32_t expected = 0; expected < kScans;) {
    if (const auto scan = input.nextScan()) {
      ASSERT_EQ(scan->header.sequence_number, expected);
      ++expected;
    } else {
      std::this_thread::yield();
    }
  }
  source.join();
  EXPECT_EQ(input.droppedScans(), 0U);
  EXPECT_EQ(input.nextScan(), nullptr);
}

// The IMU queue blocks the same way at its own depth.
TEST(SensorInput, BlockingInputHoldsImuSamplesToo) {
  auto sensors = std::make_shared<FakeSensors>();
  mslam::SensorInput input(
      sensors, sensors, true, true,
      {.max_queued_imus = 4, .overflow = mslam::SensorOverflow::Block});

  constexpr std::uint64_t kSamples = 40;
  std::thread source([&] {
    for (std::uint64_t i = 0; i < kSamples; ++i) {
      sensors->pushImu(stationaryImu(kScanBaseNs + i));
    }
  });

  for (std::uint64_t expected = 0; expected < kSamples;) {
    if (const auto imu = input.nextImu()) {
      ASSERT_EQ(imu->header.timestamp, kScanBaseNs + expected);
      ++expected;
    } else {
      std::this_thread::yield();
    }
  }
  source.join();
  EXPECT_FALSE(input.nextImu().has_value());
}

// Destroying the input releases a source that is waiting for room, so a
// blocked recording never deadlocks shutdown.
TEST(SensorInput, DestructionReleasesABlockedSource) {
  auto sensors = std::make_shared<FakeSensors>();
  auto input = std::make_unique<mslam::SensorInput>(
      sensors, sensors, true, true,
      mslam::SensorInputOptions{.overflow = mslam::SensorOverflow::Block});

  std::atomic<int> returned{0};
  // FakeSensors holds its mutex while it calls back, like msensor's
  // CallbackSlot: the destructor's unregistration waits for the callback.
  std::thread source([&] {
    sensors->pushScan(timedRoomScan(0));
    returned.store(1);
    sensors->pushScan(timedRoomScan(1)); // waits: the queue is full
    returned.store(2);
  });
  while (returned.load() < 1) {
    std::this_thread::sleep_for(1ms);
  }
  std::this_thread::sleep_for(20ms);
  ASSERT_EQ(returned.load(), 1);

  input.reset(); // must not hang
  source.join();
  EXPECT_EQ(returned.load(), 2);
  EXPECT_FALSE(sensors->hasScanCallback());
}

// A depth of 0 is raised to 1, and the default policy still replaces.
TEST(SensorInput, DefaultPolicyStillReplacesTheOldestScan) {
  auto sensors = std::make_shared<FakeSensors>();
  mslam::SensorInput input(sensors, sensors, true, true,
                           {.max_queued_scans = 0});
  sensors->pushScan(timedRoomScan(0));
  sensors->pushScan(timedRoomScan(1));
  EXPECT_EQ(input.droppedScans(), 1U);
  const auto scan = input.nextScan();
  ASSERT_TRUE(scan);
  EXPECT_EQ(scan->header.sequence_number, 1U);
}

} // namespace
