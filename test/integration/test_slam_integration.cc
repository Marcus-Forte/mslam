// End-to-end replay of a recording through the SLAM, twice:
//
//  - synchronously through SlamPipeline, the way bench_slam does it (the
//    reference: no threads, no clock);
//  - through the live loop (SlamRunner + SensorInput) fed by the playback
//    driver on its own thread;
//  - and, in a build with TBB, synchronously again with slam.num_threads = 4,
//    which must give the same pose as the first replay to the last bit.
//
// The live run is deterministic: the sensor input blocks the playback thread
// instead of dropping (SensorOverflow::Block), the playback is unpaced, the
// IMU wait is far longer than any scheduling stall, and the run starts and
// stops on events, not on sleeps. Nothing in it is decided by the wall clock,
// so it gives the same pose on every run and under any load, and that pose
// must equal the synchronous one bit for bit.

#include "msensor/conversions/lidar_imu_conversions.hh"
#include "msensor/recorder/recording_driver.hh"
#include "msensor/recorder/scan_player.hh"
#include "mslam/config/JsonConfig.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/ImuScanSync.hh"
#include "mslam/slam/MapFactory.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/SlamPipeline.hh"
#include "mslam/slam/SlamRunner.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/utils/Parallel.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace {

using namespace std::chrono_literals;

// A loop-closure / drift check: the recording walks a closed loop, so the final
// position should be back near the origin. Only translation is asserted here.
// This is LiDAR-inertial odometry without loop closure, so heading is not
// expected to return to zero (the living_room_garden run ends ~0.49 rad off).
constexpr double k_max_origin_offset_m = 0.1;

// The live run takes no decision on the wall clock as long as the playback
// thread is never starved for this long. A scan whose IMU samples never arrive
// (a recording that ends on a scan) is processed after this wait.
constexpr auto k_max_imu_wait = 30s;
// Fails the test instead of hanging if the live run cannot finish.
constexpr auto k_watchdog = 180s;

/// Pose-graph strategy: where the corrected (map frame) trajectory ended.
struct PoseGraphOutcome {
  mslam::Pose3D map_pose;
  std::size_t keyframes = 0;
  std::size_t accepted_loops = 0;
  std::size_t rejected_loops = 0;
};

/// What one replay ended with.
struct ReplayResult {
  mslam::Pose3D pose; ///< odom frame (the estimator's own pose)
  std::optional<PoseGraphOutcome> graph; ///< only with the pose-graph strategy
  mslam::RegistrationStats registration;
  mslam::ImuSyncStats sync;
  std::size_t scans = 0; ///< scans the pipeline processed
};

std::shared_ptr<mslam::VoxelHashMap>
makeMap(const mslam::SlamConfiguration &config) {
  return mslam::makeVoxelHashMap(config.map_parameters,
                                 config.parameters.backend);
}

ReplayResult resultOf(const mslam::Slam &slam,
                      const mslam::SlamPipeline &pipeline) {
  ReplayResult result{slam.getPose(), std::nullopt, slam.registrationStats(),
                      pipeline.syncStats(), pipeline.timings().scans};
  if (const auto graph = pipeline.backendResult()) {
    const Eigen::Affine3d map = pipeline.mapFromOdom() * slam.getTransform();
    const Eigen::Matrix3d rotation = map.linear();
    const Eigen::Vector3d position = map.translation();
    const Eigen::Vector3d euler = mslam::rotationToEuler(rotation);
    PoseGraphOutcome outcome;
    outcome.map_pose << position.x(), position.y(), position.z(), euler.x(),
        euler.y(), euler.z();
    outcome.keyframes = graph->stats.keyframes;
    outcome.accepted_loops = graph->stats.candidates_accepted;
    outcome.rejected_loops = graph->stats.candidates_rejected;
    result.graph = outcome;
  }
  return result;
}

/// The reference: the recording read in file order on the calling thread.
/// Scans wait until the IMU stream has reached their end, as in bench_slam.
ReplayResult replaySynchronously(const mslam::SlamConfiguration &config,
                                 const std::shared_ptr<spdlog::logger> &logger,
                                 const std::filesystem::path &recording) {
  mslam::Slam slam(logger, config, makeMap(config));
  mslam::NullSlamOutput output;
  mslam::SlamPipeline pipeline(slam, config, logger, output);
  msensor::ScanPlayer player{recording};

  std::deque<mslam::Scan> pending;
  const auto drain = [&](bool flush) {
    while (!pending.empty() && (flush || pipeline.imuCovers(pending.front()))) {
      pipeline.processSyncedScan(pending.front());
      pending.pop_front();
    }
  };

  while (player.next()) {
    const auto &entry = player.getLastEntry();
    if (entry.entry_case() == sensors::RecordingEntry::kImu) {
      pipeline.pushImu(msensor::fromProtobuf(entry.imu()));
      drain(false);
    } else if (entry.entry_case() == sensors::RecordingEntry::kScan) {
      auto scan = msensor::fromProtobuf(entry.scan());
      if (scan.points.empty()) {
        continue;
      }
      pending.push_back(std::move(scan));
      drain(false);
    }
  }
  drain(true);
  pipeline.finish(); // pose graph: seal the last keyframe (no-op otherwise)
  return resultOf(slam, pipeline);
}

/// Counts the scans the pipeline finished: it publishes exactly one map
/// increment per processed scan.
class CountingOutput final : public mslam::ISlamOutput {
public:
  void updatePose(const mslam::Pose3D &) override {}
  void updateTransformedScan(mslam::PointCloud) override {}
  using mslam::ISlamOutput::updateMapIncrement;
  void updateMapIncrement(mslam::PointCloud) override {
    processed_scans_.fetch_add(1);
  }
  void reset() override {}

  std::size_t processedScans() const { return processed_scans_.load(); }

private:
  std::atomic<std::size_t> processed_scans_{0};
};

/// The playback driver as the loop's sensors. It starts the playback when the
/// loop has registered its callbacks (so no sample is emitted into the void and
/// no sleep is needed), and counts the scans it hands over.
class ReplaySource final : public msensor::ILidar, public msensor::IImu {
public:
  ReplaySource(std::shared_ptr<msensor::RecordingSensorDriver> driver,
               bool with_lidar, bool with_imu)
      : driver_(std::move(driver)), needs_lidar_(with_lidar),
        needs_imu_(with_imu) {}

  void init() override { driver_->init(); }
  void startSampling() override {} // started by the registrations
  void stopSampling() override { driver_->stopSampling(); }

  void setScanCallback(ScanCallback callback) override {
    if (!callback) {
      driver_->setScanCallback({});
      return;
    }
    driver_->setScanCallback(
        [this, callback = std::move(callback)](msensor::Scan3DI scan) {
          if (!scan.points.empty()) {
            emitted_scans_.fetch_add(1); // the loop skips empty scans
          }
          callback(std::move(scan));
        });
    has_lidar_ = true;
    startWhenRegistered();
  }

  void setImuCallback(ImuCallback callback) override {
    const bool registering = static_cast<bool>(callback);
    driver_->setImuCallback(std::move(callback));
    if (registering) {
      has_imu_ = true;
      startWhenRegistered();
    }
  }

  /// True once the whole recording was handed over.
  bool finished() const { return driver_->isFinished(); }
  std::size_t emittedScans() const { return emitted_scans_.load(); }

private:
  // Both registrations come from the loop thread, one after the other.
  void startWhenRegistered() {
    if (started_ || (needs_lidar_ && !has_lidar_) ||
        (needs_imu_ && !has_imu_)) {
      return;
    }
    started_ = true;
    driver_->startSampling();
  }

  std::shared_ptr<msensor::RecordingSensorDriver> driver_;
  bool needs_lidar_;
  bool needs_imu_;
  bool has_lidar_ = false;
  bool has_imu_ = false;
  bool started_ = false;
  std::atomic<std::size_t> emitted_scans_{0};
};

/// A live run and what it observed besides the result.
struct LiveRun {
  ReplayResult result;
  std::size_t emitted_scans = 0;
  std::size_t dropped_scans = 0;
  std::size_t imu_wait_timeouts = 0;
  bool finished = false; ///< false: the watchdog ended the run
};

LiveRun replayThroughTheLiveLoop(const mslam::SlamConfiguration &config,
                                 const std::shared_ptr<spdlog::logger> &logger,
                                 const std::filesystem::path &recording) {
  // Speed 0: unpaced. The blocking sensor input holds the playback thread
  // back, so the replay runs exactly as fast as the SLAM takes the data.
  auto driver =
      std::make_shared<msensor::RecordingSensorDriver>(recording, 0.0);
  auto source = std::make_shared<ReplaySource>(driver, config.with_lidar,
                                               config.with_imu);
  source->init();

  mslam::Slam slam(logger, config, makeMap(config));
  CountingOutput output;
  mslam::SlamPipeline pipeline(slam, config, logger, output);

  mslam::SlamRunnerOptions options;
  options.max_imu_wait = k_max_imu_wait;
  options.input.overflow = mslam::SensorOverflow::Block;
  mslam::SlamRunner runner(pipeline, source, source, config, logger, options);

  // The live loop stays alive when the sensors go quiet, so end the run when
  // every scan the playback handed over has been processed.
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> finished{false};
  std::thread watcher([&] {
    const auto deadline = std::chrono::steady_clock::now() + k_watchdog;
    while (std::chrono::steady_clock::now() < deadline) {
      if (source->finished() &&
          output.processedScans() == source->emittedScans()) {
        finished.store(true);
        break;
      }
      std::this_thread::sleep_for(1ms);
    }
    stop_requested.store(true);
  });

  runner.run(stop_requested);
  watcher.join();
  source->stopSampling();

  LiveRun run;
  run.result = resultOf(slam, pipeline);
  run.emitted_scans = source->emittedScans();
  run.dropped_scans = runner.droppedScans();
  run.imu_wait_timeouts = runner.imuWaitTimeouts();
  run.finished = finished.load();
  return run;
}

double positionNorm(const mslam::Pose3D &pose) {
  return std::sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
}

std::ostream &operator<<(std::ostream &os, const ReplayResult &r) {
  const auto precision = os.precision(9);
  os << std::fixed << "pose=[" << r.pose[0] << " " << r.pose[1] << " "
     << r.pose[2] << " " << r.pose[3] << " " << r.pose[4] << " " << r.pose[5]
     << "] scans=" << r.scans << " imu_path=" << r.registration.imu_scans
     << " lidar_only=" << r.registration.lidar_only_scans
     << " uncovered=" << r.sync.scans_uncovered << " splits=" << r.sync.splits;
  os.unsetf(std::ios::fixed);
  os.precision(precision);
  return os;
}

class SlamIntegration : public ::testing::Test {
protected:
  // Both replays run once for the whole suite.
  static void SetUpTestSuite() {
    const std::filesystem::path source_dir{MSLAM_SOURCE_DIR};
    const auto recording = source_dir / "test/data/living_room_garden.pbscan";
    if (!std::filesystem::exists(recording)) {
      return; // every test skips
    }

    // Test-scoped config, deliberately independent of config/mslam.jsonc.
    mslam::JsonConfig json_config(source_dir / "test/integration/mslam.jsonc");
    json_config.load();
    const auto config = json_config.getConfig();

    auto logger = spdlog::default_logger();
    logger->set_level(spdlog::level::off);

    synchronous_ = replaySynchronously(config, logger, recording);
    // The same recording with the pose-graph back end (synchronous: no
    // thread, so the result does not depend on timing).
    mslam::JsonConfig graph_json(source_dir /
                                 "test/integration/mslam_pose_graph.jsonc");
    graph_json.load();
    pose_graph_ =
        replaySynchronously(graph_json.getConfig(), logger, recording);
    live_ = replayThroughTheLiveLoop(config, logger, recording);
    if (mslam::ParallelExecutor::available()) {
      auto parallel_config = config;
      parallel_config.parameters.num_threads = 4;
      four_threads_ = replaySynchronously(parallel_config, logger, recording);
    }

    std::cout << "Synchronous: " << *synchronous_ << "\n"
              << "Live loop:   " << live_->result
              << " dropped_scans=" << live_->dropped_scans
              << " imu_wait_timeouts=" << live_->imu_wait_timeouts << "\n";
    std::cout << "Pose graph:  " << *pose_graph_;
    if (const auto &g = pose_graph_->graph) {
      std::cout << " map_pose=[" << g->map_pose[0] << " " << g->map_pose[1]
                << " " << g->map_pose[2] << " " << g->map_pose[3] << " "
                << g->map_pose[4] << " " << g->map_pose[5]
                << "] keyframes=" << g->keyframes
                << " loops accepted=" << g->accepted_loops
                << " rejected=" << g->rejected_loops;
    }
    std::cout << "\n";
    if (four_threads_.has_value()) {
      std::cout << "4 threads:   " << *four_threads_ << "\n";
    }
  }

  static void TearDownTestSuite() {
    synchronous_.reset();
    live_.reset();
    four_threads_.reset();
    pose_graph_.reset();
  }

  void SetUp() override {
    if (!live_.has_value()) {
      GTEST_SKIP() << "Missing recording test/data/living_room_garden.pbscan "
                      "(untracked sample data)";
    }
  }

  static std::optional<ReplayResult> synchronous_;
  static std::optional<LiveRun> live_;
  /// The synchronous replay with slam.num_threads = 4 (absent without TBB).
  static std::optional<ReplayResult> four_threads_;
  /// The synchronous replay with strategy = pose_graph.
  static std::optional<ReplayResult> pose_graph_;
};

std::optional<ReplayResult> SlamIntegration::synchronous_;
std::optional<LiveRun> SlamIntegration::live_;
std::optional<ReplayResult> SlamIntegration::four_threads_;
std::optional<ReplayResult> SlamIntegration::pose_graph_;

TEST_F(SlamIntegration, LivingRoomGardenReturnsToOrigin) {
  ASSERT_TRUE(live_->finished) << "The live run did not consume the recording";

  const auto &pose = live_->result.pose;
  const double position_error_m = positionNorm(pose);
  const double rotation_error_rad =
      std::sqrt(pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5]);
  RecordProperty("final_position_error_m", std::to_string(position_error_m));
  RecordProperty("final_rotation_error_rad",
                 std::to_string(rotation_error_rad));

  EXPECT_LT(position_error_m, k_max_origin_offset_m)
      << "Final position error is " << position_error_m << " m (pos=["
      << pose[0] << ", " << pose[1] << ", " << pose[2] << "])";
  EXPECT_LT(positionNorm(synchronous_->pose), k_max_origin_offset_m);
}

// The live loop over a lossless source must lose nothing and take no timing
// decision.
TEST_F(SlamIntegration, LiveLoopLosesNothing) {
  ASSERT_TRUE(live_->finished) << "The live run did not consume the recording";

  EXPECT_EQ(live_->dropped_scans, 0U);
  EXPECT_EQ(live_->imu_wait_timeouts, 0U);
  EXPECT_EQ(live_->result.scans, live_->emitted_scans);
  EXPECT_EQ(live_->result.sync.stale_dropped, 0U);
  EXPECT_EQ(live_->result.sync.overflow_dropped, 0U);
}

// ... and then it is the same computation as the synchronous replay: the same
// scans with the same IMU windows in the same order, so the same pose to the
// last bit. This is what makes the run above repeatable.
TEST_F(SlamIntegration, LiveLoopMatchesTheSynchronousReplay) {
  ASSERT_TRUE(live_->finished) << "The live run did not consume the recording";
  const auto &live = live_->result;
  const auto &sync = *synchronous_;

  EXPECT_EQ(live.scans, sync.scans);
  EXPECT_EQ(live.registration.imu_scans, sync.registration.imu_scans);
  EXPECT_EQ(live.registration.lidar_only_scans,
            sync.registration.lidar_only_scans);
  EXPECT_EQ(live.sync.scans, sync.sync.scans);
  EXPECT_EQ(live.sync.scans_uncovered, sync.sync.scans_uncovered);
  EXPECT_EQ(live.sync.splits, sync.sync.splits);

  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(live.pose[i], sync.pose[i])
        << "pose[" << i << "] differs by " << live.pose[i] - sync.pose[i];
  }
}

// The parallel correspondence search and normal estimation write each result
// into its own slot and keep every order-dependent step sequential, so the
// thread count cannot change the result. A difference here is a bug in that
// code, not floating-point noise.
TEST_F(SlamIntegration, FourThreadsMatchTheSynchronousReplay) {
  if (!four_threads_.has_value()) {
    GTEST_SKIP() << "built without TBB";
  }
  const auto &parallel = *four_threads_;
  const auto &sync = *synchronous_;

  EXPECT_EQ(parallel.scans, sync.scans);
  EXPECT_EQ(parallel.registration.imu_scans, sync.registration.imu_scans);
  EXPECT_EQ(parallel.registration.lidar_only_scans,
            sync.registration.lidar_only_scans);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(parallel.pose[i], sync.pose[i])
        << "pose[" << i << "] differs by " << parallel.pose[i] - sync.pose[i];
  }
}

// The pose-graph strategy leaves the odometry alone and corrects it on the
// output side. The recording does not end exactly where it started (the
// odometry ends 0.0436 m from the origin and nobody knows the true end), so the
// corrected pose is not expected to go to the origin: it must stay close to the
// odometry pose, and a jump bigger than the threshold means a bad loop edge.
// Measured (dev container, aarch64, GCC 16): the map pose ends 0.4 mm from the
// odom pose and 0.0435 m from the origin, with 32 accepted and 0 rejected loop
// verifications; the threshold leaves a factor of 50.
constexpr double k_max_map_odom_offset_m = 0.02;

TEST_F(SlamIntegration, PoseGraphClosesTheLoop) {
  ASSERT_TRUE(pose_graph_->graph.has_value());
  const auto &graph = *pose_graph_->graph;
  const auto &sync = *synchronous_;
  RecordProperty("pose_graph_loops_accepted",
                 std::to_string(graph.accepted_loops));

  EXPECT_GE(graph.accepted_loops, 1U);
  EXPECT_GT(graph.keyframes, 10U);

  // The odom frame is the odometry's, bit for bit.
  EXPECT_EQ(pose_graph_->scans, sync.scans);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(pose_graph_->pose[i], sync.pose[i])
        << "odom pose[" << i << "] differs by "
        << pose_graph_->pose[i] - sync.pose[i];
  }

  // The map frame stays near it and inside the origin gate.
  const Eigen::Vector3d odom_position(sync.pose[0], sync.pose[1], sync.pose[2]);
  const Eigen::Vector3d map_position(graph.map_pose[0], graph.map_pose[1],
                                     graph.map_pose[2]);
  EXPECT_LT((map_position - odom_position).norm(), k_max_map_odom_offset_m);
  EXPECT_LT(positionNorm(graph.map_pose), k_max_origin_offset_m);
}

} // namespace
