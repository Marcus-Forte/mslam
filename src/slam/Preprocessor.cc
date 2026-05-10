#include "slam/Preprocessor.hh"
#include <pcl/filters/voxel_grid.h>

namespace mslam {

namespace {
float squaredDistanceToCenter(const msensor::Point3 &point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}
} // namespace

Preprocessor::Preprocessor(const PreProcessor &config) : config_(config) {}

std::shared_ptr<msensor::Scan3D>
Preprocessor::preprocess(const msensor::Scan3D &input) const {
  if (config_.min_distance_to_center <= 0.0F) {
    return downsample(input);
  }

  auto filtered_scan = removePointsNearCenter(input);
  return downsample(*filtered_scan);
}

std::shared_ptr<msensor::Scan3D>
Preprocessor::downsample(const msensor::Scan3D &input) const {
  auto filtered_scan = std::make_shared<msensor::Scan3D>();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(config_.voxel_size, config_.voxel_size,
                         config_.voxel_size);

  voxel_grid.setInputCloud(input.points);
  voxel_grid.filter(*filtered_scan->points);

  filtered_scan->timestamp = input.timestamp;
  return filtered_scan;
}

std::shared_ptr<msensor::Scan3D>
Preprocessor::removePointsNearCenter(const msensor::Scan3D &input) const {
  auto filtered_scan = std::make_shared<msensor::Scan3D>();
  filtered_scan->timestamp = input.timestamp;

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
} // namespace mslam