#include "slam/Preprocessor.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

mslam::Point makePoint(float x, float y, float z) {
  mslam::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST(Preprocessor, RemovesPointsCloserThanConfiguredCenterDistance) {
  mslam::PreProcessor config;
  config.voxel_size = 0.01F;
  config.min_distance_to_center = 1.0F;

  mslam::Preprocessor preprocessor(config);
  mslam::Scan input;
  input.header.timestamp = 1234;
  input.points->push_back(makePoint(0.0F, 0.0F, 0.0F));
  input.points->push_back(makePoint(0.5F, 0.0F, 0.0F));
  input.points->push_back(makePoint(1.0F, 0.0F, 0.0F));
  input.points->push_back(makePoint(0.0F, 0.0F, -1.2F));

  const auto filtered = preprocessor.removePointsNearCenter(input);

  ASSERT_EQ(filtered->header.timestamp, input.header.timestamp);
  ASSERT_EQ(filtered->points->size(), 2U);
  EXPECT_FLOAT_EQ(filtered->points->at(0).x, 1.0F);
  EXPECT_FLOAT_EQ(filtered->points->at(1).z, -1.2F);
}

TEST(Preprocessor, DownsampleCanKeepOriginalPointPositions) {
  mslam::PreProcessor config;
  config.voxel_size = 1.0F;
  config.downsample_filter = mslam::DownsampleFilter::VoxelHash;

  mslam::Preprocessor preprocessor(config);
  mslam::Scan input;
  input.header.timestamp = 55U;
  input.points->push_back(makePoint(0.1F, 0.2F, 0.3F));
  input.points->push_back(makePoint(0.8F, 0.7F, 0.6F));
  input.points->push_back(makePoint(1.2F, 1.3F, 1.4F));

  const auto filtered = preprocessor.downsample(input);

  ASSERT_EQ(filtered->header.timestamp, input.header.timestamp);
  ASSERT_EQ(filtered->points->size(), 2U);
  EXPECT_TRUE(std::any_of(filtered->points->cbegin(), filtered->points->cend(),
                          [](const auto &point) {
                            return point.x == 0.1F && point.y == 0.2F &&
                                   point.z == 0.3F;
                          }));
  EXPECT_TRUE(std::any_of(filtered->points->cbegin(), filtered->points->cend(),
                          [](const auto &point) {
                            return point.x == 1.2F && point.y == 1.3F &&
                                   point.z == 1.4F;
                          }));
}

// --- Deskew tests ---

TEST(Preprocessor, DeskewIsIdentityWhenMotionIsIdentity) {
  mslam::PreProcessor config;
  config.points_per_second = 100000;

  mslam::Preprocessor preprocessor(config);
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points->push_back(makePoint(1.0F, 0.0F, 0.0F));
  scan.points->push_back(makePoint(0.0F, 1.0F, 0.0F));
  scan.points->push_back(makePoint(0.0F, 0.0F, 1.0F));

  const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  const auto result = preprocessor.deskew(scan, identity);

  ASSERT_EQ(result->points->size(), 3U);
  EXPECT_NEAR(result->points->at(0).x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points->at(0).y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points->at(1).y, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points->at(2).z, 1.0F, 1e-5F);
}

TEST(Preprocessor, DeskewCompensatesPureTranslation) {
  mslam::PreProcessor config;
  config.points_per_second = 2; // 2 pts/s => each point 0.5s apart

  mslam::Preprocessor preprocessor(config);
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points->push_back(makePoint(0.0F, 0.0F, 0.0F));
  scan.points->push_back(makePoint(0.0F, 0.0F, 0.0F));

  // Relative motion: 1m translation in X
  Eigen::Affine3d delta = Eigen::Affine3d::Identity();
  delta.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  const auto result = preprocessor.deskew(scan, delta);

  ASSERT_EQ(result->points->size(), 2U);
  // Last point (stamp=1) is identity => stays at (0,0,0)
  // First point (stamp=0) gets exp(-1 * log(delta)) applied
  // = inverse(delta) => translated by (-1,0,0)
  EXPECT_NEAR(result->points->at(0).x, -1.0F, 1e-4F);
  EXPECT_NEAR(result->points->at(0).y, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points->at(1).x, 0.0F, 1e-5F);
  EXPECT_NEAR(result->points->at(1).y, 0.0F, 1e-5F);
}

TEST(Preprocessor, DeskewCompensatesPureRotation) {
  mslam::PreProcessor config;
  config.points_per_second = 2;

  mslam::Preprocessor preprocessor(config);
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points->push_back(makePoint(1.0F, 0.0F, 0.0F));
  scan.points->push_back(makePoint(1.0F, 0.0F, 0.0F));

  // Relative motion: 90 degree rotation around Z
  Eigen::Affine3d delta = Eigen::Affine3d::Identity();
  delta.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();

  const auto result = preprocessor.deskew(scan, delta);

  ASSERT_EQ(result->points->size(), 2U);
  // Last point (stamp=1): identity correction => (1,0,0)
  EXPECT_NEAR(result->points->at(1).x, 1.0F, 1e-5F);
  EXPECT_NEAR(result->points->at(1).y, 0.0F, 1e-5F);
  // First point (stamp=0): full inverse rotation (-90 deg)
  // (1,0,0) rotated by -90 deg around Z => (0,-1,0)
  EXPECT_NEAR(result->points->at(0).x, 0.0F, 1e-4F);
  EXPECT_NEAR(result->points->at(0).y, -1.0F, 1e-4F);
}

TEST(Preprocessor, DeskewThrowsWhenPointsPerSecondIsZero) {
  mslam::PreProcessor config;
  config.points_per_second = 0;

  mslam::Preprocessor preprocessor(config);
  mslam::Scan scan;
  scan.header.timestamp = 0;
  scan.points->push_back(makePoint(1.0F, 0.0F, 0.0F));

  const Eigen::Affine3d delta = Eigen::Affine3d::Identity();
  EXPECT_THROW(preprocessor.deskew(scan, delta), std::runtime_error);
}

} // namespace