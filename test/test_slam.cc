#include "mslam/slam/Slam.hh"

#include "mslam/slam/Transform.hh"

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <sstream>

#include <gtest/gtest.h>

namespace {

constexpr double k_gravity_mps2 = 9.80665;

std::shared_ptr<spdlog::logger> makeSilentLogger() {
  auto spdlog_logger = spdlog::default_logger();
  spdlog_logger->set_level(spdlog::level::off);
  return spdlog_logger;
}

class StaticMap final : public mslam::IMap {
public:
  mslam::PointCloud addScan(const mslam::PointCloud &scan) override {
    return scan;
  }

  Neighbor getClosestNeighbor(const mslam::Point &query) const override {
    return {query, 0.0F};
  }

  std::size_t findClosestNeighbors(const mslam::Point & /*query*/,
                                   std::span<Neighbor> /*out*/) const override {
    return 0;
  }

  const mslam::PointCloud &getPointCloudRepresentation() const override {
    return points_;
  }

  void clear() override { points_.clear(); }

private:
  mslam::PointCloud points_;
};

TEST(Slam, PredictIntegratesAllAngularAxes) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);

  msensor::IMUData imu{};
  imu.header.timestamp = 1'000'000'000ULL;
  slam.Predict(imu);

  imu.gx = 1.5F;
  imu.gy = -0.5F;
  imu.gz = 0.25F;
  imu.header.timestamp = 2'000'000'000ULL;
  slam.Predict(imu);

  const auto pose = slam.getPose();
  EXPECT_DOUBLE_EQ(pose[3], 1.5);
  EXPECT_DOUBLE_EQ(pose[4], -0.5);
  EXPECT_DOUBLE_EQ(pose[5], 0.25);
}

TEST(Slam, PredictIntegratesRotationOnSo3) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);

  // First sample aligns gravity (level attitude) and seeds the integrator.
  msensor::IMUData imu{};
  imu.header.timestamp = 1'000'000'000ULL;
  imu.az = k_gravity_mps2;
  slam.Predict(imu);

  // Rotate about the non-principal axis (1, 0, 1) for one second.
  imu.gx = 1.0F;
  imu.gz = 1.0F;
  imu.header.timestamp = 2'000'000'000ULL;
  slam.Predict(imu);

  const Eigen::Vector3d omega(1.0, 0.0, 1.0);
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(omega.norm(), omega.normalized()).toRotationMatrix();

  const auto pose = slam.getPose();
  const Eigen::Matrix3d actual =
      mslam::toAffine(0.0, 0.0, 0.0, pose[3], pose[4], pose[5]).linear();
  EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-9);
}

TEST(Slam, PredictPreintegratesLinearAcceleration) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);

  msensor::IMUData imu{};
  imu.header.timestamp = 1'000'000'000ULL;
  imu.az = k_gravity_mps2;
  slam.Predict(imu);

  imu.ax = 1.0F;
  imu.ay = -2.0F;
  imu.az = k_gravity_mps2;
  imu.header.timestamp = 2'000'000'000ULL;
  slam.Predict(imu);

  const auto pose = slam.getPose();
  EXPECT_NEAR(pose[0], 0.5, 1e-6);
  EXPECT_NEAR(pose[1], -1.0, 1e-6);
  EXPECT_NEAR(pose[2], 0.0, 1e-6);
}

TEST(Slam, PredictSeedsGravityAlignmentBeforeLinearAcceleration) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);

  constexpr double k_roll = 0.35;
  constexpr double k_pitch = -0.25;
  const Eigen::Vector3d body_gravity =
      mslam::toAffine(0.0, 0.0, 0.0, k_roll, k_pitch, 0.0)
          .linear()
          .transpose() *
      Eigen::Vector3d(0.0, 0.0, k_gravity_mps2);

  msensor::IMUData imu{};
  imu.header.timestamp = 1'000'000'000ULL;
  imu.ax = static_cast<float>(body_gravity.x());
  imu.ay = static_cast<float>(body_gravity.y());
  imu.az = static_cast<float>(body_gravity.z());
  slam.Predict(imu);

  imu.header.timestamp = 2'000'000'000ULL;
  slam.Predict(imu);

  const auto pose = slam.getPose();
  EXPECT_NEAR(pose[0], 0.0, 1e-6);
  EXPECT_NEAR(pose[1], 0.0, 1e-6);
  EXPECT_NEAR(pose[2], 0.0, 1e-6);
  EXPECT_NEAR(pose[3], k_roll, 1e-6);
  EXPECT_NEAR(pose[4], k_pitch, 1e-6);
}

TEST(Slam, PredictUsesConfiguredGravityScaleForUnitGravityImu) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::SlamConfiguration config;
  config.imu_acceleration_scale = k_gravity_mps2;
  mslam::Slam slam(logger, config, map);

  msensor::IMUData imu{};
  imu.header.timestamp = 1'000'000'000ULL;
  imu.az = 1.0F;
  slam.Predict(imu);

  imu.header.timestamp = 2'000'000'000ULL;
  slam.Predict(imu);

  const auto pose = slam.getPose();
  EXPECT_NEAR(pose[0], 0.0, 1e-6);
  EXPECT_NEAR(pose[1], 0.0, 1e-6);
  EXPECT_NEAR(pose[2], 0.0, 1e-6);
  EXPECT_NEAR(pose[3], 0.0, 1e-6);
  EXPECT_NEAR(pose[4], 0.0, 1e-6);
}

// LiDAR-only path (the default configuration, `with_imu = false`): the scan
// re-anchors the state on the map and the velocity estimate is dropped on
// purpose, so the IMU clock restarts too: the first sample after the scan only
// re-seeds it.
TEST(Slam, LidarOnlyUpdateResetsImuClockAtScanBoundary) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);

  msensor::IMUData imu{};
  imu.header.timestamp = 1'000'000'000ULL;
  imu.az = k_gravity_mps2;
  slam.Predict(imu);

  imu.ax = 1.0F;
  imu.az = k_gravity_mps2;
  imu.header.timestamp = 2'000'000'000ULL;
  slam.Predict(imu);

  msensor::Scan3DI scan;
  slam.Update(scan);

  imu.header.timestamp = 3'000'000'000ULL;
  slam.Predict(imu);

  const auto pose_after_first_imu = slam.getPose();
  EXPECT_NEAR(pose_after_first_imu[0], 0.5, 1e-6);

  imu.header.timestamp = 4'000'000'000ULL;
  slam.Predict(imu);

  const auto pose_after_second_imu = slam.getPose();
  EXPECT_NEAR(pose_after_second_imu[0], 1.0, 1e-6);
}

// IMU path: only the preintegrator restarts at the scan. The IMU clock keeps
// running, so the interval between the last sample before a scan and the first
// one after it is integrated (it used to be lost: ~10% of the IMU time).
TEST(Slam, ImuUpdateKeepsTheImuClockAcrossTheScanBoundary) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::SlamConfiguration config;
  config.with_imu = true;
  mslam::Slam slam(logger, config, map);

  msensor::IMUData imu{};
  imu.az = k_gravity_mps2;
  imu.ax = 1.0F;
  const auto feed = [&](std::uint64_t seconds) {
    imu.header.timestamp = seconds * 1'000'000'000ULL;
    slam.Predict(imu);
  };

  // Samples at 1 s .. 4 s, a scan after the 2 s and after the 4 s sample.
  feed(1);
  feed(2);
  slam.Update(msensor::Scan3DI{});
  EXPECT_NEAR(slam.registrationStats().preintegrated_seconds, 1.0, 1e-12);
  feed(3);
  feed(4);
  slam.Update(msensor::Scan3DI{});

  // The scans' preintegrators together cover the whole span of the samples,
  // including the 2 s -> 3 s interval across the first scan.
  const auto &stats = slam.registrationStats();
  EXPECT_EQ(stats.imu_scans, 2U);
  EXPECT_EQ(stats.lidar_only_scans, 0U);
  EXPECT_NEAR(stats.preintegrated_seconds, 3.0, 1e-12);
}

// A scan without integrated IMU time registers LiDAR-only even with the IMU
// enabled, and resets the IMU state.
TEST(Slam, UpdateWithoutImuDataFallsBackToLidarOnly) {
  auto logger = makeSilentLogger();
  auto map = std::make_shared<StaticMap>();
  mslam::SlamConfiguration config;
  config.with_imu = true;
  mslam::Slam slam(logger, config, map);

  msensor::IMUData imu{};
  imu.az = k_gravity_mps2;
  imu.header.timestamp = 1'000'000'000ULL;
  slam.Predict(imu); // only seeds the clock: nothing integrated
  slam.Update(msensor::Scan3DI{});

  EXPECT_EQ(slam.registrationStats().imu_scans, 0U);
  EXPECT_EQ(slam.registrationStats().lidar_only_scans, 1U);
}

// Predict runs at the IMU rate: its timing and state lines are debug-level.
TEST(Slam, PredictLogsOnlyAtDebugLevel) {
  const auto run = [](spdlog::level::level_enum level) {
    std::ostringstream out;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(out);
    auto logger = std::make_shared<spdlog::logger>("predict_log_test", sink);
    logger->set_level(spdlog::level::off); // construction logs at info
    auto map = std::make_shared<StaticMap>();
    mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);
    logger->set_level(level);

    msensor::IMUData imu{};
    imu.az = k_gravity_mps2;
    for (std::uint64_t i = 1; i <= 3; ++i) {
      imu.header.timestamp = i * 5'000'000ULL;
      slam.Predict(imu);
    }
    return out.str();
  };

  // One-off events (gravity alignment) still log at info, the per-sample
  // timing and state lines do not.
  const auto info_output = run(spdlog::level::info);
  EXPECT_EQ(info_output.find("Slam::Predict elapsed"), std::string::npos);
  EXPECT_EQ(info_output.find("State: pos="), std::string::npos);
  const auto debug_output = run(spdlog::level::debug);
  EXPECT_NE(debug_output.find("Slam::Predict elapsed"), std::string::npos);
  EXPECT_NE(debug_output.find("State: pos="), std::string::npos);
}

// The per-scan update keeps its info-level timing.
TEST(Slam, UpdateLogsTimingAtInfoLevel) {
  std::ostringstream out;
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(out);
  auto logger = std::make_shared<spdlog::logger>("update_log_test", sink);
  logger->set_level(spdlog::level::info);
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);
  out.str("");

  slam.Update(msensor::Scan3DI{});
  EXPECT_NE(out.str().find("Slam::Update elapsed"), std::string::npos);
}

} // namespace
