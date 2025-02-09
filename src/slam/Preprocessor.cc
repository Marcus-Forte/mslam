#include "slam/Preprocessor.hh"
#include <pcl/filters/voxel_grid.h>

namespace mslam {

Preprocessor::Preprocessor(const PreProcessor &config) : config_(config) {}

std::shared_ptr<msensor::Scan3D>
Preprocessor::downsample(msensor::Scan3D &input) {
  auto filtered_scan = std::make_shared<msensor::Scan3D>();
  auto pcl_cloud = std::make_shared<msensor::PointCloud3>();

  pcl_cloud->points = std::move(input.points.points);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(config_.voxel_size, config_.voxel_size,
                         config_.voxel_size);

  voxel_grid.setInputCloud(pcl_cloud);
  voxel_grid.filter(*pcl_cloud);

  filtered_scan->points.points = std::move(pcl_cloud->points);
  filtered_scan->timestamp = input.timestamp;
  return filtered_scan;
}
} // namespace mslam