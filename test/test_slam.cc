#include "slam/Slam.hh"

#include "NullLogger.hh"

#include <gtest/gtest.h>

namespace {

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

} // namespace