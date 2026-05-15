#include "slam/Preprocessor.hh"
#include "map/VoxelHashMap.hh"
#include "moptim/PlusOperations/SE3.h"

#include <Eigen/Geometry>
#include <cmath>
#include <pcl/filters/voxel_grid.h>

namespace mslam {

namespace {
float squaredDistanceToCenter(const Point &point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}
} // namespace

std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter) {
  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header = input.header;

  if (filter == DownsampleFilter::VoxelHash) {
    VoxelHashMap voxel_map(voxel_size, 1);
    voxel_map.addScan(*input.points);
    *filtered_scan->points = voxel_map.getPointCloudRepresentation();

    return filtered_scan;
  }

  pcl::VoxelGrid<Point> voxel_grid;
  voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_grid.setInputCloud(input.points);
  voxel_grid.filter(*filtered_scan->points);

  return filtered_scan;
}

std::shared_ptr<Scan> removePointsNearCenter(const Scan &input,
                                             float min_distance) {
  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header = input.header;

  const float min_distance_squared = min_distance * min_distance;
  filtered_scan->points->reserve(input.points->size());

  for (const auto &point : input.points->points) {
    if (squaredDistanceToCenter(point) >= min_distance_squared) {
      filtered_scan->points->push_back(point);
    }
  }

  return filtered_scan;
}

std::shared_ptr<Scan> filterByIntensity(const Scan &input,
                                        float min_intensity) {

  if (min_intensity <= 0.0F) {
    return std::make_shared<Scan>(input); // shallow copy: shares point cloud
  }
  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header = input.header;

  filtered_scan->points->reserve(input.points->size());

  for (const auto &point : input.points->points) {
    if (point.intensity >= min_intensity) {
      filtered_scan->points->push_back(point);
    }
  }

  return filtered_scan;
}

std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion) {
  auto result = std::make_shared<Scan>();
  result->header = scan.header;
  const std::size_t num_points = scan.points->size();
  if (num_points == 0) {
    return result;
  }

  // SE3 log of the relative motion (constant velocity twist)
  const auto omega = moptim::se3Log(relative_motion);

  result->points->resize(num_points);
  result->points->is_dense = scan.points->is_dense;

  // Incremental composition: 2 exp calls instead of N.
  // pose_0 = exp(-omega) corresponds to stamp=0 (first point, full inverse).
  // delta_pose advances by one step in the linear interpolation.
  const Eigen::Affine3d start_pose = moptim::se3Exp(-omega);
  const Eigen::Affine3d delta_pose =
      moptim::se3Exp(omega / static_cast<double>(num_points - 1));

  Eigen::Matrix3d R = start_pose.linear();
  Eigen::Vector3d t = start_pose.translation();
  const Eigen::Matrix3d dR = delta_pose.linear();
  const Eigen::Vector3d dt = delta_pose.translation();

  for (std::size_t i = 0; i < num_points; ++i) {
    const auto &pt = scan.points->points[i];
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const Eigen::Vector3d p_corrected = R * p + t;

    result->points->points[i].x = static_cast<float>(p_corrected.x());
    result->points->points[i].y = static_cast<float>(p_corrected.y());
    result->points->points[i].z = static_cast<float>(p_corrected.z());
    result->points->points[i].intensity = pt.intensity;

    // Advance pose: T_{i+1} = T_i * delta
    t = R * dt + t;
    R = R * dR;
  }

  return result;
}

} // namespace mslam
