#include "slam/PointCloudIO.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path makeTemporaryPlyPath() {
  const auto timestamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("mslam_point_cloud_" + std::to_string(timestamp) + ".ply");
}

mslam::Point makePoint(float x, float y, float z, float intensity) {
  mslam::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

TEST(PointCloudIO, ReadsAsciiVerticesAndSkipsFollowingListElements) {
  const auto path = makeTemporaryPlyPath();
  {
    std::ofstream output(path);
    output << "ply\n"
              "format ascii 1.0\n"
              "element vertex 2\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "property float intensity\n"
              "element face 1\n"
              "property list uchar int vertex_indices\n"
              "end_header\n"
              "1 2 3 4\n"
              "-1.5 0 8 9\n"
              "3 0 1 2\n";
  }

  const auto cloud = mslam::readPlyPointCloud(path);
  std::filesystem::remove(path);

  ASSERT_EQ(cloud.size(), 2U);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0F);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 4.0F);
  EXPECT_FLOAT_EQ(cloud[1].x, -1.5F);
  EXPECT_FLOAT_EQ(cloud[1].z, 8.0F);
  EXPECT_FLOAT_EQ(cloud[1].intensity, 9.0F);
}

TEST(PointCloudIO, BinaryWriterRoundTripsPointFields) {
  const auto path = makeTemporaryPlyPath();
  mslam::PointCloud input;
  input.push_back(makePoint(1.25F, -2.5F, 3.75F, 42.0F));
  input.push_back(makePoint(-4.0F, 5.5F, 6.25F, 7.0F));

  mslam::writePlyPointCloudBinary(path, input);
  const auto output = mslam::readPlyPointCloud(path);
  std::filesystem::remove(path);

  ASSERT_EQ(output.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_FLOAT_EQ(output[i].x, input[i].x);
    EXPECT_FLOAT_EQ(output[i].y, input[i].y);
    EXPECT_FLOAT_EQ(output[i].z, input[i].z);
    EXPECT_FLOAT_EQ(output[i].intensity, input[i].intensity);
  }
}

} // namespace
