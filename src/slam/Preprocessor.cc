#include "mslam/slam/Preprocessor.hh"
#include "mslam/slam/Deskew.hh"
#include "mslam/slam/ComputeContext.hh"
#include "mslam/slam/DevicePreprocessor.hh"

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace mslam {

namespace {

float squaredDistanceToCenter(const Point &point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}

} // namespace

void VoxelFilter::apply(const PointCloud &in, float min_distance,
                        float voxel_size, DownsampleFilter filter,
                        float min_intensity, PointCloud &out) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    throw std::invalid_argument("Voxel size must be finite and positive");
  }

  const bool range_test = min_distance >= 0.0F;
  const float min_distance_squared = min_distance * min_distance;
  const bool intensity_test = min_intensity > 0.0F;
  const float inverse_voxel_size = 1.0F / voxel_size;

  out.clear();

  if (filter == DownsampleFilter::VoxelHash) {
    occupied_voxels_.clear();
    for (const auto &point : in) {
      if (range_test &&
          !(squaredDistanceToCenter(point) >= min_distance_squared)) {
        continue;
      }
      // The first point of a voxel represents it, even if the intensity test
      // then rejects it.
      if (!occupied_voxels_.insert(PointToVoxel(point, inverse_voxel_size))
               .second) {
        continue;
      }
      if (intensity_test && !(point.intensity >= min_intensity)) {
        continue;
      }
      out.push_back(point);
    }
    return;
  }

  voxel_slots_.clear();
  accumulators_.clear();
  for (const auto &point : in) {
    if (range_test &&
        !(squaredDistanceToCenter(point) >= min_distance_squared)) {
      continue;
    }
    const auto [it, inserted] = voxel_slots_.try_emplace(
        PointToVoxel(point, inverse_voxel_size),
        static_cast<std::uint32_t>(accumulators_.size()));
    if (inserted) {
      accumulators_.emplace_back();
    }
    auto &accumulator = accumulators_[it->second];
    accumulator.x += point.x;
    accumulator.y += point.y;
    accumulator.z += point.z;
    accumulator.intensity += point.intensity;
    ++accumulator.count;
  }

  out.reserve(accumulators_.size());
  for (const auto &accumulator : accumulators_) {
    const auto count = static_cast<double>(accumulator.count);
    Point centroid;
    centroid.x = static_cast<float>(accumulator.x / count);
    centroid.y = static_cast<float>(accumulator.y / count);
    centroid.z = static_cast<float>(accumulator.z / count);
    centroid.intensity = static_cast<float>(accumulator.intensity / count);
    if (intensity_test && !(centroid.intensity >= min_intensity)) {
      continue;
    }
    out.push_back(centroid);
  }
}

std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter) {
  auto filtered_scan = std::make_shared<Scan>();
  filtered_scan->header = input.header;
  VoxelFilter{}.apply(input.points, /*min_distance=*/-1.0F, voxel_size, filter,
                      /*min_intensity=*/0.0F, filtered_scan->points);
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
    return std::make_shared<Scan>(input); // filter disabled: plain copy
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

Preprocessor::Preprocessor(const PreProcessor &config,
                           std::shared_ptr<spdlog::logger> logger,
                           ComputeBackend backend)
    : config_(config), logger_(std::move(logger)) {
  if (backend == ComputeBackend::Sycl &&
      config_.downsample_filter == DownsampleFilter::VoxelHash) {
    device_ =
        std::make_unique<DevicePreprocessor>(executionContextFor(backend));
  }
}

Preprocessor::~Preprocessor() = default;
Preprocessor::Preprocessor(Preprocessor &&) noexcept = default;

void Preprocessor::setDeviceWidth(std::size_t width) {
  if (device_) {
    device_->setWidth(width);
  }
}

std::shared_ptr<Scan> Preprocessor::filterNearCenter(const Scan &scan) const {
  return removePointsNearCenter(scan, config_.min_distance_to_center);
}

void Preprocessor::process(const Scan &scan,
                           const std::vector<msensor::IMUData> &imu_samples,
                           Scan &out) {
  const auto deskew_start = std::chrono::steady_clock::now();
  if (device_) {
    const bool deskew = config_.deskew && config_.points_per_second > 0;
    const double delta_t =
        deskew ? 1.0 / static_cast<double>(config_.points_per_second) : 0.0;
    out.header = scan.header;
    const bool filtered = device_->process(
        scan, imu_samples, delta_t, config_.min_distance_to_center,
        config_.voxel_size, config_.min_intensity, out.points);
    if (deskew && !device_->lastScanDeskewed()) {
      logger_->warn("Deskew: no IMU samples inside the lidar range, leaving "
                    "the scan undistorted");
    }
    const auto filter_start = std::chrono::steady_clock::now();
    timings_.deskew += filter_start - deskew_start;
    if (!filtered) {
      // Voxel coordinates beyond the device table: filter on the host.
      deskewed_.assign(device_->deskewed(),
                       device_->deskewed() + scan.points.size());
      voxel_filter_.apply(deskewed_, config_.min_distance_to_center,
                          config_.voxel_size, config_.downsample_filter,
                          config_.min_intensity, out.points);
      timings_.filter += std::chrono::steady_clock::now() - filter_start;
    }
    return;
  }
  const PointCloud *source = &scan.points;
  if (config_.deskew && config_.points_per_second > 0) {
    const double delta_t = 1.0 / static_cast<double>(config_.points_per_second);
    if (deskewInto(scan, imu_samples, delta_t, logger_, deskewed_)) {
      source = &deskewed_;
    }
  }

  const auto filter_start = std::chrono::steady_clock::now();
  timings_.deskew += filter_start - deskew_start;
  out.header = scan.header;
  voxel_filter_.apply(*source, config_.min_distance_to_center,
                      config_.voxel_size, config_.downsample_filter,
                      config_.min_intensity, out.points);
  timings_.filter += std::chrono::steady_clock::now() - filter_start;
}

std::shared_ptr<Scan>
Preprocessor::process(const Scan &scan,
                      const std::vector<msensor::IMUData> &imu_samples) {
  auto result = std::make_shared<Scan>();
  process(scan, imu_samples, *result);
  return result;
}

} // namespace mslam
