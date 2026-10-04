#include "slam/Deskew.hh"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDeltaT = 0.5; // seconds between LiDAR points
constexpr uint64_t kNanoseconds = 1'000'000'000ULL;

mslam::Point makePoint(float x, float y, float z, float intensity = 0.0F) {
  mslam::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

msensor::IMUData makeImu(uint64_t timestamp_ns, float gx, float gy, float gz) {
  msensor::IMUData data{};
  data.header.timestamp = timestamp_ns;
  data.gx = gx;
  data.gy = gy;
  data.gz = gz;
  return data;
}

mslam::Scan makeScan(uint64_t timestamp_ns, std::size_t num_points) {
  mslam::Scan scan;
  scan.header.timestamp = timestamp_ns;
  scan.points.reserve(num_points);
  for (std::size_t i = 0; i < num_points; ++i) {
    scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F));
  }
  return scan;
}

TEST(DeskewWithImu, ReturnsEmptyResultForEmptyScan) {
  mslam::Scan scan;
  scan.header.timestamp = 0;
  const std::vector<msensor::IMUData> imu;

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  EXPECT_TRUE(result->points.empty());
  EXPECT_EQ(result->header.timestamp, scan.header.timestamp);
}

TEST(DeskewWithImu, ThrowsForNonPositiveDeltaTime) {
  const auto scan = makeScan(0, 2);
  const std::vector<msensor::IMUData> imu{makeImu(0, 0.0F, 0.0F, 0.0F)};

  EXPECT_THROW(mslam::deskew(scan, imu, 0.0), std::invalid_argument);
  EXPECT_THROW(mslam::deskew(scan, imu, -0.1), std::invalid_argument);
  EXPECT_THROW(
      mslam::deskew(scan, imu, std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST(DeskewWithImu, LeavesScanUnchangedWhenNoImuSamplesFit) {
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F, 10.0F));
  scan.points.push_back(makePoint(0.0F, 1.0F, 0.0F, 20.0F));

  const std::vector<msensor::IMUData> imu;

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), scan.points.size());
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    EXPECT_FLOAT_EQ(result->points[i].x, scan.points[i].x);
    EXPECT_FLOAT_EQ(result->points[i].y, scan.points[i].y);
    EXPECT_FLOAT_EQ(result->points[i].z, scan.points[i].z);
    EXPECT_FLOAT_EQ(result->points[i].intensity, scan.points[i].intensity);
  }
}

TEST(DeskewWithImu, IgnoresImuSamplesOutsideScanDuration) {
  const auto scan = makeScan(kNanoseconds, 3); // spans [1e9, 2e9] ns
  const float gz = static_cast<float>(M_PI / 2.0);

  const std::vector<msensor::IMUData> imu{
      // Before the scan window: ignored.
      makeImu(0, 0.0F, 0.0F, static_cast<float>(M_PI)),
      // Inside the scan window.
      makeImu(kNanoseconds, 0.0F, 0.0F, gz),
      makeImu(2 * kNanoseconds, 0.0F, 0.0F, gz),
      // After the scan window: ignored.
      makeImu(3 * kNanoseconds, 0.0F, 0.0F, static_cast<float>(M_PI))};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  EXPECT_NEAR(result->points[0].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].x, std::cos(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[1].y, std::sin(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[2].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 1.0F, 1e-5F);
}

TEST(DeskewWithImu, LogsTimingAndSampleCount) {
  const auto scan = makeScan(0, 3); // spans [0, 1e9] ns
  const std::vector<msensor::IMUData> imu{
      makeImu(0, 0.0F, 0.0F, 0.0F), makeImu(kNanoseconds / 2, 0.0F, 0.0F, 0.0F),
      makeImu(2 * kNanoseconds, 0.0F, 0.0F, 0.0F)}; // outside the window

  std::ostringstream log_stream;
  auto logger = std::make_shared<spdlog::logger>(
      "deskew-test",
      std::make_shared<spdlog::sinks::ostream_sink_mt>(log_stream));
  logger->set_level(spdlog::level::info);

  const auto result = mslam::deskew(scan, imu, kDeltaT, logger);
  logger->flush();

  ASSERT_EQ(result->points.size(), 3U);
  const std::string log = log_stream.str();
  EXPECT_NE(log.find("Deskew took"), std::string::npos);
  EXPECT_NE(log.find("2 of 3 IMU samples fit the lidar range"),
            std::string::npos);
}

TEST(DeskewWithImu, KeepsPointsAndMetadataWhenGyroIsZero) {
  mslam::Scan scan;
  scan.header.timestamp = 1234;
  scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F, 10.0F));
  scan.points.push_back(makePoint(0.0F, 2.0F, 0.0F, 20.0F));
  scan.points.push_back(makePoint(0.0F, 0.0F, 3.0F, 30.0F));

  const std::vector<msensor::IMUData> imu{
      makeImu(1234, 0.0F, 0.0F, 0.0F),
      makeImu(1234 + kNanoseconds, 0.0F, 0.0F, 0.0F)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), scan.points.size());
  EXPECT_EQ(result->header.timestamp, scan.header.timestamp);
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    EXPECT_FLOAT_EQ(result->points[i].x, scan.points[i].x);
    EXPECT_FLOAT_EQ(result->points[i].y, scan.points[i].y);
    EXPECT_FLOAT_EQ(result->points[i].z, scan.points[i].z);
    EXPECT_FLOAT_EQ(result->points[i].intensity, scan.points[i].intensity);
  }
}

TEST(DeskewWithImu, RotatesLaterPointsWithConstantAngularVelocity) {
  const auto scan = makeScan(0, 3); // points at t = 0, 0.5, 1.0 s
  const float gz = static_cast<float>(M_PI / 2.0); // 90 deg over the scan
  const std::vector<msensor::IMUData> imu{
      makeImu(0, 0.0F, 0.0F, gz), makeImu(kNanoseconds, 0.0F, 0.0F, gz)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  // First point is in the scan-start frame and is unchanged.
  EXPECT_NEAR(result->points[0].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, 0.0F, 1e-5F);
  // Mid point: half the rotation (45 deg).
  EXPECT_NEAR(result->points[1].x, std::cos(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[1].y, std::sin(M_PI / 4.0), 1e-5F);
  // Last point: the full rotation (90 deg).
  EXPECT_NEAR(result->points[2].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 1.0F, 1e-5F);
}

TEST(DeskewWithImu, HoldsAngularVelocityBeforeFirstSample) {
  const auto scan = makeScan(0, 2); // points at t = 0, 0.5 s
  const float gz = static_cast<float>(M_PI / 2.0);
  const std::vector<msensor::IMUData> imu{
      makeImu(kNanoseconds / 2, 0.0F, 0.0F, gz)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 2U);
  EXPECT_NEAR(result->points[0].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, 0.0F, 1e-5F);
  // From the scan start to 0.5 s the first sample's rate is held.
  EXPECT_NEAR(result->points[1].x, std::cos(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[1].y, std::sin(M_PI / 4.0), 1e-5F);
}

TEST(DeskewWithImu, UsesEachSampleOverItsOwnInterval) {
  const auto scan = makeScan(0, 3); // points at t = 0, 0.5, 1.0 s
  const std::vector<msensor::IMUData> imu{
      makeImu(0, 0.0F, 0.0F, 0.0F),
      makeImu(kNanoseconds / 2, 0.0F, 0.0F, static_cast<float>(M_PI))};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  // First interval uses the zero-rate sample.
  EXPECT_NEAR(result->points[1].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].y, 0.0F, 1e-5F);
  // Second interval integrates the pi rad/s sample over 0.5 s => 90 deg.
  EXPECT_NEAR(result->points[2].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 1.0F, 1e-5F);
}

TEST(DeskewWithImu, RotatesAboutYAxis) {
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points.push_back(makePoint(0.0F, 0.0F, 1.0F));
  scan.points.push_back(makePoint(0.0F, 0.0F, 1.0F));

  const std::vector<msensor::IMUData> imu{
      makeImu(0, 0.0F, static_cast<float>(M_PI), 0.0F),
      makeImu(kNanoseconds / 2, 0.0F, static_cast<float>(M_PI), 0.0F)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 2U);
  // (0, 0, 1) rotated 90 deg about +Y becomes (1, 0, 0).
  EXPECT_NEAR(result->points[1].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].z, 0.0F, 1e-5F);
}

TEST(DeskewWithImu, AcceptsUnsortedSamples) {
  const auto scan = makeScan(0, 3);
  const float gz = static_cast<float>(M_PI / 2.0);
  const std::vector<msensor::IMUData> imu{makeImu(kNanoseconds, 0.0F, 0.0F, gz),
                                          makeImu(0, 0.0F, 0.0F, gz)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  EXPECT_NEAR(result->points[2].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 1.0F, 1e-5F);
}

} // namespace
