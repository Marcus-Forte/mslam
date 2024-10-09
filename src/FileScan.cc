#include "FileScan.hh"
#include "points.pb.h"
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <thread>

namespace {
mslam::PointCloud2D fromProto(const lidar::PointCloud3 &msg) {
  mslam::PointCloud2D pointcloud;
  pointcloud.points.reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    pointcloud.points.emplace_back(pt.x(), pt.y());
  }
  pointcloud.timestamp = msg.timestamp();
  return pointcloud;
}
} // namespace
namespace mslam {
FileScan::FileScan(const std::string &file_scan) {
  if (!std::filesystem::exists(file_scan)) {
    throw std::runtime_error("file " + file_scan + "does not exist");
  }

  scan_file_.open(file_scan, std::ios::binary | std::ios::in);
  lidar::PointCloud3 pointcloud;
  size_t bytes;
  scan_file_ >> bytes;

  std::string data(bytes, 0);
  scan_file_.read(&data[0], bytes);

  if (!pointcloud.ParseFromString(data)) {
    throw std::runtime_error("invalid scanfile " + file_scan);
  }
  scan_time_ = pointcloud.timestamp();
}
PointCloud2D FileScan::getScan(bool blocking) {
  if (scan_file_.peek() == EOF) {
    // Loop back
    scan_file_.seekg(0);
  }
  size_t bytes;
  scan_file_ >> bytes;
  std::string data(bytes, 0);
  scan_file_.read(&data[0], bytes);
  lidar::PointCloud3 pointcloud;
  if (!pointcloud.ParseFromString(data)) {
    throw std::runtime_error("invalid read");
  }
  // Simulate sample time.
  if (blocking) {
    const auto delta_ms = pointcloud.timestamp() - scan_time_;
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms));
    scan_time_ = pointcloud.timestamp();
  }

  return fromProto(pointcloud);
}

} // namespace mslam