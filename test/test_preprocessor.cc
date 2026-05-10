#include "slam/Preprocessor.hh"

#include <gtest/gtest.h>

#include <algorithm>

namespace {

msensor::Point3 makePoint(float x, float y, float z) {
  msensor::Point3 point;
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
  msensor::Scan3D input;
  input.timestamp = 1234;
  input.points->push_back(makePoint(0.0F, 0.0F, 0.0F));
  input.points->push_back(makePoint(0.5F, 0.0F, 0.0F));
  input.points->push_back(makePoint(1.0F, 0.0F, 0.0F));
  input.points->push_back(makePoint(0.0F, 0.0F, -1.2F));

  const auto filtered = preprocessor.removePointsNearCenter(input);

  ASSERT_EQ(filtered->timestamp, input.timestamp);
  ASSERT_EQ(filtered->points->size(), 2U);
  EXPECT_FLOAT_EQ(filtered->points->at(0).x, 1.0F);
  EXPECT_FLOAT_EQ(filtered->points->at(1).z, -1.2F);
}

TEST(Preprocessor, DownsampleCanKeepOriginalPointPositions) {
  mslam::PreProcessor config;
  config.voxel_size = 1.0F;
  config.downsample_filter = mslam::DownsampleFilter::VoxelHash;

  mslam::Preprocessor preprocessor(config);
  msensor::Scan3D input;
  input.timestamp = 55U;
  input.points->push_back(makePoint(0.1F, 0.2F, 0.3F));
  input.points->push_back(makePoint(0.8F, 0.7F, 0.6F));
  input.points->push_back(makePoint(1.2F, 1.3F, 1.4F));

  const auto filtered = preprocessor.downsample(input);

  ASSERT_EQ(filtered->timestamp, input.timestamp);
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

} // namespace