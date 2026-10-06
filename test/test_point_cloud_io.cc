#include "mslam/slam/PointCloudIO.hh"

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

} // namespace
