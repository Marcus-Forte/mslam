#include "slam/Preprocessor.hh"
#include <pcl/filters/voxel_grid.h>

namespace mslam {

Preprocessor::Preprocessor(const PreProcessor &config) : config_(config) {}

std::shared_ptr<msensor::Scan3D>
Preprocessor::downsample(msensor::Scan3D &input) {
  auto filtered_scan = std::make_shared<msensor::Scan3D>();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(config_.voxel_size, config_.voxel_size,
                         config_.voxel_size);

  voxel_grid.setInputCloud(input.points);
  voxel_grid.filter(*filtered_scan->points);

  filtered_scan->timestamp = input.timestamp;
  return filtered_scan;
}
} // namespace mslam