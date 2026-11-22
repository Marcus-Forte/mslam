// DevicePreprocessor: deskew, range test and voxel_hash downsampling in one
// kernel, against deskewInto() and VoxelFilter.
#include "mslam/slam/Deskew.hh"
#include "mslam/slam/DevicePreprocessor.hh"
#include "mslam/slam/Preprocessor.hh"

#include <bit>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <random>

namespace {

constexpr double kDeltaT = 5e-6; // 200000 points per second
constexpr std::uint64_t kStartNs = 1'000'000'000ULL;

mslam::Scan makeScan(std::size_t points, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> coordinate(-6.0F, 6.0F);
  std::uniform_real_distribution<float> intensity(0.0F, 120.0F);
  mslam::Scan scan;
  scan.header.timestamp = kStartNs;
  for (std::size_t i = 0; i < points; ++i) {
    scan.points.push_back({coordinate(rng), coordinate(rng),
                           0.3F * coordinate(rng), intensity(rng)});
  }
  return scan;
}

// IMU every 5 ms over the scan's 48 ms, turning about all axes.
std::vector<msensor::IMUData> makeImu() {
  std::vector<msensor::IMUData> samples;
  for (int k = -1; k < 12; ++k) {
    msensor::IMUData sample{};
    sample.header.timestamp = kStartNs + static_cast<std::uint64_t>(
                                             std::max(0, k) * 5'000'000 + 1300);
    sample.gx = 0.3F + 0.05F * static_cast<float>(k);
    sample.gy = -0.2F;
    sample.gz = 1.1F - 0.04F * static_cast<float>(k);
    samples.push_back(sample);
  }
  return samples;
}

bool sameBits(float a, float b) {
  return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
}

bool samePoint(const mslam::Point &a, const mslam::Point &b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z) &&
         sameBits(a.intensity, b.intensity);
}

} // namespace

TEST(DevicePreprocessor, WithoutDeskewItIsTheHostVoxelFilter) {
  const mslam::Scan scan = makeScan(9600, 1);
  mslam::PointCloud expected;
  mslam::VoxelFilter{}.apply(scan.points, 1.0F, 0.1F,
                             mslam::DownsampleFilter::VoxelHash, 50.0F,
                             expected);
  mslam::DevicePreprocessor device;
  for (const std::size_t width : {1, 3, 11}) {
    device.setWidth(width);
    mslam::PointCloud got;
    ASSERT_TRUE(device.process(scan, {}, 0.0, 1.0F, 0.1F, 50.0F, got));
    ASSERT_EQ(got.size(), expected.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
      ASSERT_TRUE(samePoint(got[i], expected[i]));
    }
  }
}

TEST(DevicePreprocessor, DeskewFollowsTheHostDeskewAndFilter) {
  const mslam::Scan scan = makeScan(9600, 2);
  const auto imu = makeImu();
  mslam::PointCloud host_deskewed;
  ASSERT_TRUE(mslam::deskewInto(scan, imu, kDeltaT, nullptr, host_deskewed));

  mslam::DevicePreprocessor device;
  device.setWidth(4);
  mslam::PointCloud got;
  ASSERT_TRUE(device.process(scan, imu, kDeltaT, 1.0F, 0.1F, 50.0F, got));
  ASSERT_TRUE(device.lastScanDeskewed());
  const mslam::Point *deskewed = device.deskewed();
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    if (i < mslam::DevicePreprocessor::kBlock) {
      // Block 0 runs deskewInto()'s product from the scan start.
      ASSERT_TRUE(samePoint(deskewed[i], host_deskewed[i])) << i;
    } else {
      ASSERT_NEAR(deskewed[i].x, host_deskewed[i].x, 2e-6F) << i;
      ASSERT_NEAR(deskewed[i].y, host_deskewed[i].y, 2e-6F) << i;
      ASSERT_NEAR(deskewed[i].z, host_deskewed[i].z, 2e-6F) << i;
    }
  }
  // The selection is VoxelFilter's on the same deskewed points.
  mslam::PointCloud expected;
  mslam::VoxelFilter{}.apply(
      mslam::PointCloud(deskewed, deskewed + scan.points.size()), 1.0F, 0.1F,
      mslam::DownsampleFilter::VoxelHash, 50.0F, expected);
  ASSERT_EQ(got.size(), expected.size());
  for (std::size_t i = 0; i < got.size(); ++i) {
    ASSERT_TRUE(samePoint(got[i], expected[i]));
  }
}

TEST(DevicePreprocessor, ResultsDoNotDependOnTheWidthAndRepeat) {
  const mslam::Scan scan = makeScan(9600, 3);
  const auto imu = makeImu();
  mslam::DevicePreprocessor device;
  device.setWidth(1);
  mslam::PointCloud reference;
  ASSERT_TRUE(device.process(scan, imu, kDeltaT, 1.0F, 0.1F, 50.0F, reference));
  for (const std::size_t width : {2, 5, 11, 64, 1}) {
    device.setWidth(width);
    mslam::PointCloud got;
    ASSERT_TRUE(device.process(scan, imu, kDeltaT, 1.0F, 0.1F, 50.0F, got));
    ASSERT_EQ(got.size(), reference.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
      ASSERT_TRUE(samePoint(got[i], reference[i]));
    }
  }
}

TEST(DevicePreprocessor, FarPointsFallBackToTheHostFilter) {
  mslam::Scan scan = makeScan(1000, 4);
  scan.points[10].x = 5000.0F; // voxel 50000: beyond the 16-bit table keys
  mslam::DevicePreprocessor device;
  mslam::PointCloud got;
  EXPECT_FALSE(device.process(scan, {}, 0.0, 1.0F, 0.1F, 50.0F, got));

  // Preprocessor handles it with the host filter.
  mslam::PreProcessor config;
  config.min_distance_to_center = 1.0F;
  config.voxel_size = 0.1F;
  config.min_intensity = 50.0F;
  config.downsample_filter = mslam::DownsampleFilter::VoxelHash;
  config.deskew = false;
  mslam::Preprocessor preprocessor(config, spdlog::default_logger());
  mslam::Scan out;
  preprocessor.process(scan, {}, out);
  mslam::PointCloud expected;
  mslam::VoxelFilter{}.apply(scan.points, 1.0F, 0.1F,
                             mslam::DownsampleFilter::VoxelHash, 50.0F,
                             expected);
  ASSERT_EQ(out.points.size(), expected.size());
}
