#include "slam/Preprocessor.hh"
#include "map/VoxelHashMap.hh"
#include "moptim/PlusOperations/SE3.h"

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>

namespace mslam {

namespace {
struct PointAccumulator {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  std::size_t count = 0;
};

float squaredDistanceToCenter(const Point &point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}

void downsampleToCentroids(const PointCloud &input, float voxel_size,
                           PointCloud &output) {
  std::map<std::array<int, 3>, PointAccumulator> voxels;
  const float inverse_voxel_size = 1.0F / voxel_size;

  for (const auto &point : input) {
    const auto voxel = PointToVoxel(point, inverse_voxel_size);
    auto &accumulator =
        voxels[std::array<int, 3>{voxel.x(), voxel.y(), voxel.z()}];
    accumulator.x += point.x;
    accumulator.y += point.y;
    accumulator.z += point.z;
    accumulator.intensity += point.intensity;
    ++accumulator.count;
  }

  output.reserve(voxels.size());
  for (const auto &entry : voxels) {
    const auto &accumulator = entry.second;
    const auto count = static_cast<double>(accumulator.count);
    Point centroid;
    centroid.x = static_cast<float>(accumulator.x / count);
    centroid.y = static_cast<float>(accumulator.y / count);
    centroid.z = static_cast<float>(accumulator.z / count);
    centroid.intensity = static_cast<float>(accumulator.intensity / count);
    output.push_back(centroid);
  }
}
} // namespace

std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    throw std::invalid_argument("Voxel size must be finite and positive");
  }

  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header = input.header;

  if (filter == DownsampleFilter::VoxelHash) {
    VoxelHashMap voxel_map(voxel_size, 1);
    voxel_map.addScan(input.points);
    filtered_scan->points = voxel_map.getPointCloudRepresentation();

    return filtered_scan;
  }

  downsampleToCentroids(input.points, voxel_size, filtered_scan->points);

  return filtered_scan;
}

std::shared_ptr<Scan> removePointsNearCenter(const Scan &input,
                                             float min_distance) {
  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header = input.header;

  const float min_distance_squared = min_distance * min_distance;
  filtered_scan->points.reserve(input.points.size());

  for (const auto &point : input.points) {
    if (squaredDistanceToCenter(point) >= min_distance_squared) {
      filtered_scan->points.push_back(point);
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

  filtered_scan->points.reserve(input.points.size());

  for (const auto &point : input.points) {
    if (point.intensity >= min_intensity) {
      filtered_scan->points.push_back(point);
    }
  }

  return filtered_scan;
}

std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion) {
  auto result = std::make_shared<Scan>();
  result->header = scan.header;
  const std::size_t num_points = scan.points.size();
  if (num_points == 0) {
    return result;
  }

  const auto omega = moptim::se3Log(relative_motion);

  result->points.resize(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    const double stamp =
        static_cast<double>(i) / static_cast<double>(num_points - 1);
    const Eigen::Affine3d pose = moptim::se3Exp((stamp - 1.0) * omega);

    const auto &pt = scan.points[i];
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const Eigen::Vector3d p_corrected = pose * p;

    result->points[i].x = static_cast<float>(p_corrected.x());
    result->points[i].y = static_cast<float>(p_corrected.y());
    result->points[i].z = static_cast<float>(p_corrected.z());
    result->points[i].intensity = pt.intensity;
  }

  return result;
}

std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion,
                             unsigned int scan_rate, double delta_t) {
  auto result = std::make_shared<Scan>();
  result->header = scan.header;
  const std::size_t num_points = scan.points.size();
  if (num_points == 0) {
    return result;
  }

  const auto omega_ref = moptim::se3Log(relative_motion);

  // Scale twist: relative_motion was observed over delta_t seconds,
  // but this scan spans scan_duration seconds at the known scan_rate.
  const double scan_duration =
      static_cast<double>(num_points - 1) / static_cast<double>(scan_rate);
  const auto omega = omega_ref * (scan_duration / delta_t);

  result->points.resize(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    const double stamp =
        static_cast<double>(i) / static_cast<double>(num_points - 1);
    const Eigen::Affine3d pose = moptim::se3Exp((stamp - 1.0) * omega);

    const auto &pt = scan.points[i];
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const Eigen::Vector3d p_corrected = pose * p;

    result->points[i].x = static_cast<float>(p_corrected.x());
    result->points[i].y = static_cast<float>(p_corrected.y());
    result->points[i].z = static_cast<float>(p_corrected.z());
    result->points[i].intensity = pt.intensity;
  }

  return result;
}

} // namespace mslam

namespace mslam {

Preprocessor::Preprocessor(const PreProcessor &config) : config_(config) {}

std::shared_ptr<Scan> Preprocessor::filterNearCenter(const Scan &scan) const {
  return removePointsNearCenter(scan, config_.min_distance_to_center);
}

std::shared_ptr<Scan>
Preprocessor::process(const Scan &scan, const Eigen::Affine3d &last_delta,
                      uint64_t last_scan_timestamp_ns) const {
  std::shared_ptr<Scan> result;

  if (config_.deskew_mode != DeskewMode::Off) {
    if (config_.points_per_second > 0 && last_scan_timestamp_ns > 0) {
      const double delta_t =
          static_cast<double>(scan.header.timestamp - last_scan_timestamp_ns) *
          1e-9;
      result = delta_t > 0.0 ? deskew(scan, last_delta,
                                      config_.points_per_second, delta_t)
                             : deskew(scan, last_delta);
    } else {
      result = deskew(scan, last_delta);
    }
  } else {
    result = std::make_shared<Scan>(scan);
  }

  result = removePointsNearCenter(*result, config_.min_distance_to_center);
  result = downsample(*result, config_.voxel_size, config_.downsample_filter);
  result = filterByIntensity(*result, config_.min_intensity);

  return result;
}

} // namespace mslam
