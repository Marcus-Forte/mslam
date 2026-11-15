#include "mslam/slam/PointCloudIO.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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

std::string readFile(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

const std::string kHeader10 =
    "ply\nformat binary_little_endian 1.0\nelement vertex ";
const std::string kHeaderProps =
    "\nproperty float x\nproperty float y\nproperty float z\n"
    "property float intensity\nend_header\n";

mslam::PointCloud makeCloud(std::size_t n, float base) {
  mslam::PointCloud cloud(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto f = static_cast<float>(i);
    cloud[i] = {base + f * 0.1F, -base - f * 1.0e-7F, 1.0F / (f + 3.0F),
                f * 2.5F};
  }
  return cloud;
}

TEST(PointCloudIOWrite, WritesHeaderExactSizeAndRoundTripsBitForBit) {
  const auto path = makeTemporaryPlyPath();
  const auto cloud = makeCloud(5, 1.5F);
  const auto written = mslam::writePlyPointCloud(path, cloud);
  const std::string bytes = readFile(path);
  std::filesystem::remove(path);

  const std::string header = kHeader10 + "5" + kHeaderProps;
  ASSERT_EQ(bytes.compare(0, header.size(), header), 0);
  EXPECT_EQ(bytes.size(), header.size() + 5 * 16);
  EXPECT_EQ(written, bytes.size());
  EXPECT_EQ(mslam::plyFileSize(5), bytes.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    float row[4];
    std::memcpy(row, bytes.data() + header.size() + i * 16, 16);
    EXPECT_EQ(std::memcmp(row, &cloud[i], 16), 0) << "point " << i;
  }
}

TEST(PointCloudIOWrite, ConcatenatesChunksInOrder) {
  const std::vector<mslam::PointCloud> chunks = {
      makeCloud(3, 1.0F), {}, makeCloud(2, 7.0F)};
  std::ostringstream out;
  const auto written = mslam::writePlyPointCloud(out, chunks);
  const std::string bytes = out.str();
  const std::string header = kHeader10 + "5" + kHeaderProps;
  ASSERT_EQ(bytes.size(), header.size() + 5 * 16);
  EXPECT_EQ(written, bytes.size());
  EXPECT_EQ(bytes.compare(0, header.size(), header), 0);
  EXPECT_EQ(std::memcmp(bytes.data() + header.size(), chunks[0].data(), 48), 0);
  EXPECT_EQ(
      std::memcmp(bytes.data() + header.size() + 48, chunks[2].data(), 32), 0);
}

TEST(PointCloudIOWrite, EmptyCloudIsAValidFileWithZeroVertices) {
  const auto path = makeTemporaryPlyPath();
  mslam::writePlyPointCloud(path, {});
  const std::string bytes = readFile(path);
  std::filesystem::remove(path);
  EXPECT_EQ(bytes, kHeader10 + "0" + kHeaderProps);
}

TEST(PointCloudIOWrite, WrittenFileIsReadBackByReadPlyPointCloud) {
  const auto path = makeTemporaryPlyPath();
  const auto cloud = makeCloud(4, -2.0F);
  mslam::writePlyPointCloud(path, cloud);
  const auto read = mslam::readPlyPointCloud(path);
  std::filesystem::remove(path);
  ASSERT_EQ(read.size(), cloud.size());
  EXPECT_EQ(std::memcmp(read.data(), cloud.data(), 16 * cloud.size()), 0);
}

TEST(PointCloudIOWrite, UnwritablePathThrowsInsteadOfCrashing) {
  const auto path = std::filesystem::temp_directory_path() /
                    "mslam_no_such_directory_xyz" / "map.ply";
  EXPECT_THROW(mslam::writePlyPointCloud(path, makeCloud(1, 0.0F)),
               std::runtime_error);
}

} // namespace
