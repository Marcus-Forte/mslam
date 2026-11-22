#include "mslam/slam/PointCloudIO.hh"

#include "msensor/file/file.hh"
#include "msensor/recorder/scan_recorder.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// Unique per process and per call: ctest runs the tests of this binary as
// parallel processes, and a clock reading alone can collide between them.
std::filesystem::path makeTemporaryPlyPath() {
  static std::atomic<unsigned> counter{0};
  const auto timestamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("mslam_point_cloud_" + std::to_string(::getpid()) + "_" +
          std::to_string(counter++) + "_" + std::to_string(timestamp) +
          ".ply");
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

// A scan with `count` points; point k of scan `id` is (id, k, id + k, k).
msensor::Scan3DI makeScan(std::uint32_t id, std::size_t count) {
  msensor::Scan3DI scan;
  scan.header = msensor::Header{1'000ULL * id, id};
  for (std::size_t k = 0; k < count; ++k) {
    const auto f = static_cast<float>(k);
    scan.points.push_back(
        {static_cast<float>(id), f, static_cast<float>(id) + f, f});
  }
  return scan;
}

void record(const std::filesystem::path &path,
            const std::vector<msensor::Scan3DI> &scans, bool imu_between) {
  msensor::ScanRecorder recorder(std::make_shared<msensor::File>());
  recorder.start(path.string());
  for (const auto &scan : scans) {
    recorder.record(scan);
    if (imu_between) {
      recorder.record(
          msensor::IMUData{scan.header, 0.0F, 0.0F, 9.81F, 0.1F, 0.2F, 0.3F});
    }
  }
  recorder.stop();
}

std::filesystem::path makeTemporaryPath(const std::string &extension) {
  auto path = makeTemporaryPlyPath();
  path.replace_extension(extension);
  return path;
}

TEST(PointCloudIOPbscan, ConcatenatesTheScansInOrderAndSkipsImuEntries) {
  const auto path = makeTemporaryPath(".pbscan");
  const std::vector<msensor::Scan3DI> scans = {makeScan(1, 3), makeScan(2, 0),
                                               makeScan(3, 4)};
  {
    msensor::ScanRecorder recorder(std::make_shared<msensor::File>());
    recorder.start(path.string());
    recorder.record(
        msensor::IMUData{msensor::Header{1, 0}, 0, 0, 9.81F, 0, 0, 0});
    recorder.record(scans[0]);
    recorder.record(
        msensor::IMUData{msensor::Header{2, 1}, 1, 1, 9.81F, 1, 1, 1});
    recorder.record(scans[1]);
    recorder.record(scans[2]);
    recorder.record(
        msensor::IMUData{msensor::Header{3, 2}, 2, 2, 9.81F, 2, 2, 2});
    recorder.stop();
  }

  const auto cloud = mslam::readPbscanPointCloud(path);
  std::filesystem::remove(path);

  ASSERT_EQ(cloud.size(), 7U);
  std::size_t index = 0;
  for (const auto &scan : scans) {
    for (const auto &point : scan.points) {
      EXPECT_EQ(std::memcmp(&cloud[index], &point, sizeof(point)), 0)
          << "point " << index;
      ++index;
    }
  }
}

TEST(PointCloudIOPbscan, AnEmptyRecordingIsAnEmptyCloud) {
  const auto path = makeTemporaryPath(".pbscan");
  record(path, {}, false);
  const auto cloud = mslam::readPbscanPointCloud(path);
  std::filesystem::remove(path);
  EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudIOPbscan, AMissingFileThrows) {
  EXPECT_THROW(mslam::readPbscanPointCloud(makeTemporaryPath(".pbscan")),
               std::runtime_error);
}

TEST(PointCloudIOPbscan, ReadPointCloudDispatchesOnTheExtension) {
  const auto ply = makeTemporaryPath(".ply");
  const auto pbscan = makeTemporaryPath(".pbscan");
  mslam::writePlyPointCloud(ply, makeCloud(5, 2.0F));
  record(pbscan, {makeScan(7, 2), makeScan(8, 3)}, true);

  EXPECT_EQ(mslam::readPointCloud(ply).size(), 5U);
  EXPECT_EQ(mslam::readPointCloud(pbscan).size(), 5U);
  std::filesystem::remove(ply);
  std::filesystem::remove(pbscan);
}

TEST(PointCloudIOPbscan, ReadPointCloudRejectsOtherExtensions) {
  const auto path = makeTemporaryPath(".xyz");
  std::ofstream(path) << "1 2 3\n";
  EXPECT_THROW(mslam::readPointCloud(path), std::runtime_error);
  EXPECT_THROW(mslam::readPointCloud("no_extension"), std::runtime_error);
  std::filesystem::remove(path);
}

} // namespace
