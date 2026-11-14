#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/Deskew.hh"
#include "mslam/slam/Preprocessor.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace {

mslam::Point makePoint(float x, float y, float z, float intensity = 0.0F) {
  mslam::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

TEST(Preprocessor, RemovesPointsCloserThanConfiguredCenterDistance) {
  mslam::Scan input;
  input.header.timestamp = 1234;
  input.points.push_back(makePoint(0.0F, 0.0F, 0.0F));
  input.points.push_back(makePoint(0.5F, 0.0F, 0.0F));
  input.points.push_back(makePoint(1.0F, 0.0F, 0.0F));
  input.points.push_back(makePoint(0.0F, 0.0F, -1.2F));

  const auto filtered = mslam::removePointsNearCenter(input, 1.0F);

  ASSERT_EQ(filtered->header.timestamp, input.header.timestamp);
  ASSERT_EQ(filtered->points.size(), 2U);
  EXPECT_FLOAT_EQ(filtered->points.at(0).x, 1.0F);
  EXPECT_FLOAT_EQ(filtered->points.at(1).z, -1.2F);
}

TEST(Preprocessor, DownsampleCanKeepOriginalPointPositions) {
  mslam::Scan input;
  input.header.timestamp = 55U;
  input.points.push_back(makePoint(0.1F, 0.2F, 0.3F, 2.0F));
  input.points.push_back(makePoint(0.8F, 0.7F, 0.6F, 3.0F));
  input.points.push_back(makePoint(1.2F, 1.3F, 1.4F, 4.0F));

  const auto filtered =
      mslam::downsample(input, 1.0F, mslam::DownsampleFilter::VoxelHash);

  ASSERT_EQ(filtered->header.timestamp, input.header.timestamp);
  ASSERT_EQ(filtered->points.size(), 2U);
  EXPECT_TRUE(std::any_of(filtered->points.cbegin(), filtered->points.cend(),
                          [](const auto &point) {
                            return point.x == 0.1F && point.y == 0.2F &&
                                   point.z == 0.3F;
                          }));
  EXPECT_TRUE(std::any_of(filtered->points.cbegin(), filtered->points.cend(),
                          [](const auto &point) {
                            return point.x == 1.2F && point.y == 1.3F &&
                                   point.z == 1.4F;
                          }));
  EXPECT_FLOAT_EQ(filtered->points.at(0).intensity, 2.0F);
  EXPECT_FLOAT_EQ(filtered->points.at(1).intensity, 4.0F);
}

TEST(Preprocessor, VoxelHashDownsamplePreservesIntensityForFiltering) {
  mslam::Scan input;
  input.points.push_back(makePoint(0.1F, 0.2F, 0.3F, 2.0F));
  input.points.push_back(makePoint(1.1F, 1.2F, 1.3F, 8.0F));

  const auto downsampled =
      mslam::downsample(input, 1.0F, mslam::DownsampleFilter::VoxelHash);
  const auto filtered = mslam::filterByIntensity(*downsampled, 5.0F);

  ASSERT_EQ(filtered->points.size(), 1U);
  EXPECT_FLOAT_EQ(filtered->points.front().intensity, 8.0F);
}

TEST(Preprocessor, VoxelGridDownsampleReturnsVoxelCentroids) {
  mslam::Scan input;
  input.points.push_back(makePoint(0.1F, 0.2F, 0.3F));
  input.points.push_back(makePoint(0.3F, 0.4F, 0.5F));
  input.points.push_back(makePoint(1.2F, 1.3F, 1.4F));

  const auto filtered =
      mslam::downsample(input, 1.0F, mslam::DownsampleFilter::VoxelGrid);

  ASSERT_EQ(filtered->points.size(), 2U);
  EXPECT_FLOAT_EQ(filtered->points.at(0).x, 0.2F);
  EXPECT_FLOAT_EQ(filtered->points.at(0).y, 0.3F);
  EXPECT_FLOAT_EQ(filtered->points.at(0).z, 0.4F);
  EXPECT_FLOAT_EQ(filtered->points.at(1).x, 1.2F);
}

TEST(Preprocessor, DownsampleRejectsNonPositiveVoxelSize) {
  mslam::Scan input;
  EXPECT_THROW(
      mslam::downsample(input, 0.0F, mslam::DownsampleFilter::VoxelGrid),
      std::invalid_argument);
}

// --- Deskew tests ---

TEST(Preprocessor, DeskewIsIdentityWhenMotionIsIdentity) {
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F));
  scan.points.push_back(makePoint(0.0F, 1.0F, 0.0F));
  scan.points.push_back(makePoint(0.0F, 0.0F, 1.0F));

  const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  const auto result = mslam::deskew(scan, identity);

  ASSERT_EQ(result->points.size(), 3U);
  EXPECT_NEAR(result->points.at(0).x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points.at(0).y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points.at(1).y, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points.at(2).z, 1.0F, 1e-5F);
}

TEST(Preprocessor, DeskewCompensatesPureTranslation) {
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points.push_back(makePoint(0.0F, 0.0F, 0.0F));
  scan.points.push_back(makePoint(0.0F, 0.0F, 0.0F));

  // Relative motion: 1m translation in X
  Eigen::Affine3d delta = Eigen::Affine3d::Identity();
  delta.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  const auto result = mslam::deskew(scan, delta);

  ASSERT_EQ(result->points.size(), 2U);
  // Last point (stamp=1) is identity => stays at (0,0,0)
  // First point (stamp=0) gets exp(-1 * log(delta)) applied
  // = inverse(delta) => translated by (-1,0,0)
  EXPECT_NEAR(result->points.at(0).x, -1.0F, 1e-4F);
  EXPECT_NEAR(result->points.at(0).y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points.at(1).x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points.at(1).y, 0.0F, 1e-5F);
}

TEST(Preprocessor, DeskewCompensatesPureRotation) {
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F));
  scan.points.push_back(makePoint(1.0F, 0.0F, 0.0F));

  // Relative motion: 90 degree rotation around Z
  Eigen::Affine3d delta = Eigen::Affine3d::Identity();
  delta.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();

  const auto result = mslam::deskew(scan, delta);

  ASSERT_EQ(result->points.size(), 2U);
  // Last point (stamp=1): identity correction => (1,0,0)
  EXPECT_NEAR(result->points.at(1).x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points.at(1).y, 0.0F, 1e-5F);
  // First point (stamp=0): full inverse rotation (-90 deg)
  // (1,0,0) rotated by -90 deg around Z => (0,-1,0)
  EXPECT_NEAR(result->points.at(0).x, 0.0F, 1e-4F);
  EXPECT_NEAR(result->points.at(0).y, -1.0F, 1e-4F);
}

TEST(Preprocessor, DeskewThrowsWhenScanIsEmpty) {
  mslam::Scan scan;
  scan.header.timestamp = 0;

  const Eigen::Affine3d delta = Eigen::Affine3d::Identity();
  const auto result = mslam::deskew(scan, delta);
  EXPECT_EQ(result->points.size(), 0U);
}

// --- Fused voxel filter ---

mslam::Scan randomScan(std::mt19937 &rng, int count) {
  std::uniform_real_distribution<float> coordinate(-6.0F, 6.0F);
  std::uniform_real_distribution<float> intensity(0.0F, 100.0F);
  mslam::Scan scan;
  scan.header.timestamp = 77;
  for (int i = 0; i < count; ++i) {
    scan.points.push_back(makePoint(coordinate(rng), coordinate(rng),
                                    coordinate(rng), intensity(rng)));
  }
  // A few exact duplicates and points at the origin.
  scan.points.push_back(scan.points.front());
  scan.points.push_back(makePoint(0.0F, 0.0F, 0.0F, 90.0F));
  return scan;
}

void expectSameCloud(const mslam::PointCloud &lhs,
                     const mslam::PointCloud &rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    ASSERT_EQ(lhs[i].x, rhs[i].x) << i;
    ASSERT_EQ(lhs[i].y, rhs[i].y) << i;
    ASSERT_EQ(lhs[i].z, rhs[i].z) << i;
    ASSERT_EQ(lhs[i].intensity, rhs[i].intensity) << i;
  }
}

TEST(Preprocessor, FusedFilterMatchesTheSequentialPipeline) {
  std::mt19937 rng(11);
  mslam::VoxelFilter filter; // reused across scans on purpose
  for (const auto downsample_filter : {mslam::DownsampleFilter::VoxelHash,
                                       mslam::DownsampleFilter::VoxelGrid}) {
    for (const float min_intensity : {0.0F, 40.0F}) {
      for (int trial = 0; trial < 5; ++trial) {
        const auto scan = randomScan(rng, 3000);
        constexpr float min_distance = 1.5F;
        constexpr float voxel_size = 0.5F;

        const auto near = mslam::removePointsNearCenter(scan, min_distance);
        const auto down =
            mslam::downsample(*near, voxel_size, downsample_filter);
        const auto expected = mslam::filterByIntensity(*down, min_intensity);

        mslam::PointCloud fused;
        filter.apply(scan.points, min_distance, voxel_size, downsample_filter,
                     min_intensity, fused);
        expectSameCloud(fused, expected->points);
      }
    }
  }
}

TEST(Preprocessor, FusedFilterRejectsInvalidVoxelSize) {
  mslam::VoxelFilter filter;
  mslam::PointCloud out;
  EXPECT_THROW(filter.apply({}, 0.0F, 0.0F, mslam::DownsampleFilter::VoxelHash,
                            0.0F, out),
               std::invalid_argument);
  EXPECT_THROW(filter.apply({}, 0.0F, std::nanf(""),
                            mslam::DownsampleFilter::VoxelGrid, 0.0F, out),
               std::invalid_argument);
}

TEST(Preprocessor, VoxelHashKeepsFirstSeenPointPerVoxelInInputOrder) {
  mslam::Scan input;
  input.points.push_back(makePoint(5.5F, 0.5F, 0.5F, 1.0F));
  input.points.push_back(makePoint(0.5F, 0.5F, 0.5F, 2.0F));
  input.points.push_back(makePoint(5.6F, 0.6F, 0.6F, 3.0F)); // same voxel as #0
  input.points.push_back(makePoint(9.5F, 0.5F, 0.5F, 4.0F));

  const auto result =
      mslam::downsample(input, 1.0F, mslam::DownsampleFilter::VoxelHash);
  ASSERT_EQ(result->points.size(), 3U);
  EXPECT_FLOAT_EQ(result->points[0].intensity, 1.0F);
  EXPECT_FLOAT_EQ(result->points[1].intensity, 2.0F);
  EXPECT_FLOAT_EQ(result->points[2].intensity, 4.0F);
}

TEST(Preprocessor, IntensityFilterRejectsAVoxelWhoseFirstPointIsDim) {
  // The voxel representative is the first point; a brighter later point in the
  // same voxel does not rescue it (as in the sequential pipeline).
  mslam::Scan input;
  input.points.push_back(makePoint(0.1F, 0.1F, 0.1F, 10.0F));
  input.points.push_back(makePoint(0.2F, 0.2F, 0.2F, 99.0F));
  mslam::VoxelFilter filter;
  mslam::PointCloud out;
  filter.apply(input.points, -1.0F, 1.0F, mslam::DownsampleFilter::VoxelHash,
               50.0F, out);
  EXPECT_TRUE(out.empty());
}

TEST(Preprocessor, ProcessReusesBuffersAndMatchesSequentialResult) {
  mslam::PreProcessor config;
  config.voxel_size = 0.5F;
  config.min_distance_to_center = 1.0F;
  config.min_intensity = 20.0F;
  config.downsample_filter = mslam::DownsampleFilter::VoxelHash;
  config.deskew = false;
  mslam::Preprocessor preprocessor(config, nullptr);

  std::mt19937 rng(21);
  mslam::Scan out;
  for (int i = 0; i < 4; ++i) {
    const auto scan = randomScan(rng, 1500);
    preprocessor.process(scan, {}, out);

    const auto expected = mslam::filterByIntensity(
        *mslam::downsample(*mslam::removePointsNearCenter(scan, 1.0F), 0.5F,
                           mslam::DownsampleFilter::VoxelHash),
        20.0F);
    EXPECT_EQ(out.header.timestamp, scan.header.timestamp);
    expectSameCloud(out.points, expected->points);

    // The allocating overload agrees.
    expectSameCloud(preprocessor.process(scan, {})->points, expected->points);
  }
}

TEST(Preprocessor, ProcessDeskewsBeforeFiltering) {
  mslam::PreProcessor config;
  config.voxel_size = 0.01F;
  config.min_distance_to_center = 0.0F;
  config.deskew = true;
  config.points_per_second = 1000; // 1 ms between points
  mslam::Preprocessor preprocessor(config, nullptr);

  mslam::Scan scan;
  scan.header.timestamp = 1'000'000'000ULL;
  for (int i = 0; i < 4; ++i) {
    scan.points.push_back(makePoint(2.0F, 0.1F * static_cast<float>(i), 0.0F));
  }
  std::vector<msensor::IMUData> imu(2);
  imu[0].header.timestamp = 1'000'000'000ULL;
  imu[1].header.timestamp = 1'003'000'000ULL;
  imu[0].gz = imu[1].gz = 1.0F; // 1 rad/s about z

  const auto result = preprocessor.process(scan, imu);
  const auto expected = mslam::deskew(scan, imu, 1e-3);
  ASSERT_EQ(result->points.size(), expected->points.size());
  for (std::size_t i = 0; i < result->points.size(); ++i) {
    EXPECT_EQ(result->points[i].x, expected->points[i].x);
    EXPECT_EQ(result->points[i].y, expected->points[i].y);
  }
  // The scan is expressed in the frame at its last point: the first point is
  // rotated, so it moved.
  EXPECT_NE(result->points.front().y, scan.points.front().y);
}

// The voxel-hash downsample keeps the same points the VoxelHashMap (one point
// per voxel) used to produce; only their order may differ.
TEST(Preprocessor, VoxelHashDownsampleKeepsTheSamePointsAsAOnePointVoxelMap) {
  std::mt19937 rng(5);
  const auto scan = randomScan(rng, 4000);

  mslam::VoxelHashMap map(0.4F, 1);
  map.addScan(scan.points);
  auto expected = map.getPointCloudRepresentation();
  auto actual =
      mslam::downsample(scan, 0.4F, mslam::DownsampleFilter::VoxelHash)->points;

  const auto order = [](const mslam::Point &a, const mslam::Point &b) {
    return std::tie(a.x, a.y, a.z, a.intensity) <
           std::tie(b.x, b.y, b.z, b.intensity);
  };
  std::sort(expected.begin(), expected.end(), order);
  std::sort(actual.begin(), actual.end(), order);
  expectSameCloud(actual, expected);
}

} // namespace
