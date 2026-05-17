#include "slam/Slam.hh"

#include "slam/Transform.hh"

#include "NullLogger.hh"

#include <gtest/gtest.h>

namespace {

constexpr double k_gravity_mps2 = 9.80665;

class StaticMap final : public mslam::IMap {
public:
  mslam::PointCloud addScan(const mslam::PointCloud &scan) override {
    return scan;
  }

  Neighbor getClosestNeighbor(const mslam::Point &query) const override {
    return {query, 0.0F};
  }

  std::vector<Neighbor> getClosestNNeighbors(const mslam::Point & /*query*/,
                                             int /*count*/) const override {
    return {};
  }

  const mslam::PointCloud &getPointCloudRepresentation() const override {
    return points_;
  }

  void clear() override { points_.points.clear(); }

private:
  mslam::PointCloud points_;
};

TEST(Slam, PredictIntegratesAllAngularAxes) {
  auto logger = std::make_shared<NullLogger>();
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

TEST(Slam, PredictPreintegratesLinearAcceleration) {
  auto logger = std::make_shared<NullLogger>();
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
  auto logger = std::make_shared<NullLogger>();
  auto map = std::make_shared<StaticMap>();
  mslam::Slam slam(logger, mslam::SlamConfiguration{}, map);

  constexpr double k_roll = 0.35;
  constexpr double k_pitch = -0.25;
  const Eigen::Vector3d body_gravity =
      toAffine(0.0, 0.0, 0.0, k_roll, k_pitch, 0.0).linear().transpose() *
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
  auto logger = std::make_shared<NullLogger>();
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

TEST(Slam, UpdateResetsImuPreintegrationAtScanBoundary) {
  auto logger = std::make_shared<NullLogger>();
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

} // namespace