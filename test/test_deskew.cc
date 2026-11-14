#include "mslam/slam/Deskew.hh"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
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

  // Scan-end frame: point i is rotated by (theta_i - 90 deg); the samples
  // outside the window (pi rad/s) do not contribute.
  ASSERT_EQ(result->points.size(), 3U);
  EXPECT_NEAR(result->points[0].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, -1.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].x, std::cos(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[1].y, -std::sin(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[2].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 0.0F, 1e-5F);
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

TEST(DeskewWithImu, RotatesEarlierPointsWithConstantAngularVelocity) {
  const auto scan = makeScan(0, 3); // points at t = 0, 0.5, 1.0 s
  const float gz = static_cast<float>(M_PI / 2.0); // 90 deg over the scan
  const std::vector<msensor::IMUData> imu{
      makeImu(0, 0.0F, 0.0F, gz), makeImu(kNanoseconds, 0.0F, 0.0F, gz)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  // The last point is in the scan-end frame and is unchanged.
  EXPECT_NEAR(result->points[2].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 0.0F, 1e-5F);
  // Mid point: the remaining half of the rotation (-45 deg).
  EXPECT_NEAR(result->points[1].x, std::cos(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[1].y, -std::sin(M_PI / 4.0), 1e-5F);
  // First point: the whole rotation (-90 deg).
  EXPECT_NEAR(result->points[0].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, -1.0F, 1e-5F);
}

// Pins the reference frame on a synthetic pure rotation: a static landmark
// seen by a sensor turning at a constant rate is measured at a different
// bearing by every point; after deskewing, every measurement must agree and
// equal the landmark in the sensor frame at the LAST point's time.
TEST(DeskewWithImu, ExpressesPointsInTheSensorFrameAtTheScanEnd) {
  constexpr double kRate = 200000.0; // points per second
  constexpr std::size_t kPoints = 20001;
  constexpr double kYawRate = 1.2; // rad/s
  constexpr std::uint64_t kStart = 1'700'000'000'000'000'000ULL;
  const double duration = static_cast<double>(kPoints - 1) / kRate; // 0.1 s

  const Eigen::Vector3d landmark(4.0, -1.5, 0.7); // static, in the start frame
  const auto sensor_to_start = [&](double t) {
    return Eigen::AngleAxisd(kYawRate * t, Eigen::Vector3d::UnitZ())
        .toRotationMatrix();
  };

  mslam::Scan scan;
  scan.header.timestamp = kStart;
  for (std::size_t i = 0; i < kPoints; ++i) {
    const Eigen::Vector3d measured =
        sensor_to_start(static_cast<double>(i) / kRate).transpose() * landmark;
    scan.points.push_back(makePoint(static_cast<float>(measured.x()),
                                    static_cast<float>(measured.y()),
                                    static_cast<float>(measured.z())));
  }

  // 200 Hz gyro covering the scan.
  std::vector<msensor::IMUData> imu;
  for (std::uint64_t t = kStart; t <= kStart + 100'000'000ULL; t += 5'000'000) {
    imu.push_back(makeImu(t, 0.0F, 0.0F, static_cast<float>(kYawRate)));
  }

  const auto result = mslam::deskew(scan, imu, 1.0 / kRate);

  const Eigen::Vector3d expected =
      sensor_to_start(duration).transpose() * landmark;
  ASSERT_EQ(result->points.size(), kPoints);
  for (const std::size_t i :
       {std::size_t{0}, kPoints / 3, kPoints / 2, kPoints - 2, kPoints - 1}) {
    EXPECT_NEAR(result->points[i].x, expected.x(), 2e-5) << i;
    EXPECT_NEAR(result->points[i].y, expected.y(), 2e-5) << i;
    EXPECT_NEAR(result->points[i].z, expected.z(), 2e-5) << i;
  }
  // The last point is returned unchanged.
  EXPECT_EQ(result->points.back().x, scan.points.back().x);
  EXPECT_EQ(result->points.back().y, scan.points.back().y);
}

TEST(DeskewWithImu, HoldsAngularVelocityBeforeFirstSample) {
  const auto scan = makeScan(0, 2); // points at t = 0, 0.5 s
  const float gz = static_cast<float>(M_PI / 2.0);
  const std::vector<msensor::IMUData> imu{
      makeImu(kNanoseconds / 2, 0.0F, 0.0F, gz)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 2U);
  // The last point is the reference.
  EXPECT_NEAR(result->points[1].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].y, 0.0F, 1e-5F);
  // From the scan start to 0.5 s the first sample's rate is held: the first
  // point is moved back by 45 deg.
  EXPECT_NEAR(result->points[0].x, std::cos(M_PI / 4.0), 1e-5F);
  EXPECT_NEAR(result->points[0].y, -std::sin(M_PI / 4.0), 1e-5F);
}

TEST(DeskewWithImu, UsesEachSampleOverItsOwnInterval) {
  const auto scan = makeScan(0, 3); // points at t = 0, 0.5, 1.0 s
  const std::vector<msensor::IMUData> imu{
      makeImu(0, 0.0F, 0.0F, 0.0F),
      makeImu(kNanoseconds / 2, 0.0F, 0.0F, static_cast<float>(M_PI))};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  // First interval uses the zero-rate sample: points 0 and 1 are in the same
  // frame, 90 deg behind the end frame (second interval: pi rad/s over 0.5 s).
  EXPECT_NEAR(result->points[2].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[1].y, -1.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, -1.0F, 1e-5F);
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
  // The last point is the reference: unchanged.
  EXPECT_NEAR(result->points[1].z, 1.0F, 1e-5F);
  // (0, 0, 1) rotated -90 deg about +Y becomes (-1, 0, 0).
  EXPECT_NEAR(result->points[0].x, -1.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].z, 0.0F, 1e-5F);
}

TEST(DeskewWithImu, AcceptsUnsortedSamples) {
  const auto scan = makeScan(0, 3);
  const float gz = static_cast<float>(M_PI / 2.0);
  const std::vector<msensor::IMUData> imu{makeImu(kNanoseconds, 0.0F, 0.0F, gz),
                                          makeImu(0, 0.0F, 0.0F, gz)};

  const auto result = mslam::deskew(scan, imu, kDeltaT);

  ASSERT_EQ(result->points.size(), 3U);
  EXPECT_NEAR(result->points[2].x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points[2].y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points[0].y, -1.0F, 1e-5F);
}

// Reference implementation: the straightforward per-point rotation increment
// that deskewInto() memoizes. The two must agree bit for bit.
std::vector<mslam::Point> referenceDeskew(const mslam::Scan &scan,
                                          std::vector<msensor::IMUData> samples,
                                          double delta_t) {
  const auto rotation_increment = [](const Eigen::Vector3d &omega, double dt) {
    const double angle = omega.norm() * dt;
    if (!(angle > 0.0) || !std::isfinite(angle)) {
      return Eigen::Matrix3d::Identity().eval();
    }
    return Eigen::AngleAxisd(angle, omega.normalized())
        .toRotationMatrix()
        .eval();
  };
  const auto omega_of = [](const msensor::IMUData &s) {
    return Eigen::Vector3d(s.gx, s.gy, s.gz);
  };

  std::sort(samples.begin(), samples.end(), [](const auto &a, const auto &b) {
    return a.header.timestamp < b.header.timestamp;
  });
  const double start = static_cast<double>(scan.header.timestamp);
  const double step = delta_t * 1e9;
  std::vector<mslam::Point> out(scan.points.size());
  // Rotation over the whole scan, per IMU interval; its inverse is the
  // starting orientation (scan-end reference frame).
  const double end = start + static_cast<double>(scan.points.size() - 1) * step;
  Eigen::Matrix3d total = Eigen::Matrix3d::Identity();
  {
    double time = start;
    Eigen::Vector3d rate = omega_of(samples.front());
    for (std::size_t k = 1; k < samples.size(); ++k) {
      const double boundary = static_cast<double>(samples[k].header.timestamp);
      total *= rotation_increment(rate, (boundary - time) / 1e9);
      time = boundary;
      rate = omega_of(samples[k]);
    }
    total *= rotation_increment(rate, (end - time) / 1e9);
  }
  Eigen::Matrix3d orientation = total.transpose();
  double previous = start;
  std::size_t index = 0;
  Eigen::Vector3d omega = omega_of(samples.front());
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const double t = start + static_cast<double>(i) * step;
    while (index + 1 < samples.size() &&
           static_cast<double>(samples[index + 1].header.timestamp) <= t) {
      const double boundary =
          static_cast<double>(samples[index + 1].header.timestamp);
      orientation *= rotation_increment(omega, (boundary - previous) / 1e9);
      previous = boundary;
      omega = omega_of(samples[++index]);
    }
    orientation *= rotation_increment(omega, (t - previous) / 1e9);
    previous = t;
    const auto &p = scan.points[i];
    const Eigen::Vector3d q = orientation * Eigen::Vector3d(p.x, p.y, p.z);
    out[i] = makePoint(static_cast<float>(q.x()), static_cast<float>(q.y()),
                       static_cast<float>(q.z()), p.intensity);
  }
  return out;
}

TEST(Deskew, MemoizedRotationIncrementsMatchTheReferenceBitForBit) {
  // Realistic epoch-scale timestamps: nanosecond values near 1.7e18 are
  // quantized in a double, so consecutive point steps differ slightly and the
  // memo sees several distinct time steps per IMU interval.
  constexpr double kDelta = 1.0 / 200000.0;
  mslam::Scan scan;
  scan.header.timestamp = 1'700'000'000'123'456'789ULL;
  std::mt19937 rng(8);
  std::uniform_real_distribution<float> coordinate(-10.0F, 10.0F);
  for (int i = 0; i < 6000; ++i) {
    scan.points.push_back(
        makePoint(coordinate(rng), coordinate(rng), coordinate(rng), 5.0F));
  }

  std::vector<msensor::IMUData> imu;
  std::uniform_real_distribution<float> rate(-2.0F, 2.0F);
  for (int i = 0; i < 8; ++i) {
    imu.push_back(makeImu(scan.header.timestamp + i * 4'000'000ULL, rate(rng),
                          rate(rng), rate(rng)));
  }
  // One stationary interval: zero angular velocity gives identity increments.
  imu[3].gx = imu[3].gy = imu[3].gz = 0.0F;

  mslam::PointCloud out;
  ASSERT_TRUE(mslam::deskewInto(scan, imu, kDelta, nullptr, out));
  const auto expected = referenceDeskew(scan, imu, kDelta);
  ASSERT_EQ(out.size(), expected.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    ASSERT_EQ(out[i].x, expected[i].x) << i;
    ASSERT_EQ(out[i].y, expected[i].y) << i;
    ASSERT_EQ(out[i].z, expected[i].z) << i;
  }

  // The scan-returning overload gives the same points.
  const auto shared = mslam::deskew(scan, imu, kDelta);
  ASSERT_EQ(shared->points.size(), out.size());
  EXPECT_EQ(shared->points[100].x, out[100].x);
}

TEST(Deskew, DeskewIntoReportsWhenThereIsNothingToDo) {
  mslam::Scan scan;
  scan.header.timestamp = 5ULL * kNanoseconds;
  scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F));
  scan.points.push_back(makePoint(0.0F, 1.0F, 0.0F));

  mslam::PointCloud out(3);
  // No IMU sample inside the scan window: the caller keeps the input points.
  EXPECT_FALSE(mslam::deskewInto(scan, {}, kDeltaT, nullptr, out));
  EXPECT_EQ(out.size(), 3U); // untouched

  mslam::Scan empty;
  EXPECT_TRUE(mslam::deskewInto(empty, {}, kDeltaT, nullptr, out));
  EXPECT_TRUE(out.empty());

  EXPECT_THROW(mslam::deskewInto(scan, {}, 0.0, nullptr, out),
               std::invalid_argument);
}

} // namespace
