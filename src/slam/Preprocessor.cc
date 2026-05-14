#include "slam/Preprocessor.hh"
#include "map/VoxelHashMap.hh"
#include "slam/Transform.hh"

#include <Eigen/Geometry>
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>

namespace mslam {

namespace {
float squaredDistanceToCenter(const Point &point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}
} // namespace

Preprocessor::Preprocessor(const PreProcessor &config) : config_(config) {}

std::shared_ptr<Scan> Preprocessor::downsample(const Scan &input) const {
  if (config_.downsample_filter == DownsampleFilter::VoxelHash) {
    VoxelHashMap voxel_map(config_.voxel_size, 1);
    voxel_map.addScan(*input.points);

    auto filtered_scan = std::make_shared<Scan>();
    filtered_scan->header.timestamp = input.header.timestamp;
    *filtered_scan->points = voxel_map.getPointCloudRepresentation();
    filtered_scan->points->is_dense = input.points->is_dense;
    return filtered_scan;
  }

  auto filtered_scan = std::make_shared<Scan>();

  pcl::VoxelGrid<Point> voxel_grid;
  voxel_grid.setLeafSize(config_.voxel_size, config_.voxel_size,
                         config_.voxel_size);

  voxel_grid.setInputCloud(input.points);
  voxel_grid.filter(*filtered_scan->points);

  filtered_scan->header = input.header;
  return filtered_scan;
}

std::shared_ptr<Scan>
Preprocessor::removePointsNearCenter(const Scan &input) const {
  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header.timestamp = input.header.timestamp;

  const float min_distance_squared =
      config_.min_distance_to_center * config_.min_distance_to_center;
  filtered_scan->points->reserve(input.points->size());

  for (const auto &point : input.points->points) {
    if (squaredDistanceToCenter(point) >= min_distance_squared) {
      filtered_scan->points->push_back(point);
    }
  }

  filtered_scan->points->width = filtered_scan->points->size();
  filtered_scan->points->height = 1;
  filtered_scan->points->is_dense = input.points->is_dense;
  return filtered_scan;
}

std::shared_ptr<Scan>
Preprocessor::deskew(const Scan &scan,
                     const Eigen::Affine3d &relative_motion) const {
  if (config_.points_per_second == 0) {
    throw std::runtime_error(
        "points_per_second must be set for lidar deskewing");
  }

  const std::size_t num_points = scan.points->size();
  if (num_points == 0) {
    auto result = std::make_shared<Scan>();
    result->header.timestamp = scan.header.timestamp;
    return result;
  }

  // SE3 log of the relative motion (constant velocity twist)
  const auto omega = se3Log(relative_motion);

  auto result = std::make_shared<Scan>();
  result->header.timestamp = scan.header.timestamp;
  result->points->resize(num_points);
  result->points->is_dense = scan.points->is_dense;

  for (std::size_t i = 0; i < num_points; ++i) {
    // Normalized timestamp in [0, 1] based on point index
    const double stamp =
        static_cast<double>(i) / static_cast<double>(num_points - 1);

    // Interpolate from identity (end of scan) back to start:
    // at stamp=1 (last point) → identity, at stamp=0 (first point) → full
    // inverse
    const Eigen::Affine3d pose = se3Exp((stamp - 1.0) * omega);

    const auto &pt = scan.points->at(i);
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const Eigen::Vector3d p_corrected = pose * p;

    result->points->at(i).x = static_cast<float>(p_corrected.x());
    result->points->at(i).y = static_cast<float>(p_corrected.y());
    result->points->at(i).z = static_cast<float>(p_corrected.z());
    result->points->at(i).intensity = pt.intensity;
  }

  return result;
}

std::shared_ptr<Scan>
Preprocessor::deskewImu(const Scan &scan,
                        const Eigen::Affine3d &imu_delta) const {
  // Same SE3 interpolation, different motion source
  return deskew(scan, imu_delta);
}
} // namespace mslam