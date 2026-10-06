#include "mslam/slam/Deskew.hh"
#include "mslam/slam/SE3.hh"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace mslam {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;
// Absorbs double rounding when comparing nanosecond timestamps (~1e9).
constexpr double kTimestampToleranceNs = 1e-3;

Eigen::Vector3d angularVelocityOf(const msensor::IMUData &sample) {
  return {sample.gx, sample.gy, sample.gz};
}

/// Body-frame rotation accumulated while turning at @p omega for @p dt seconds.
Eigen::Matrix3d rotationIncrement(const Eigen::Vector3d &omega, double dt) {
  const double angle = omega.norm() * dt;
  if (!(angle > 0.0) || !std::isfinite(angle)) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, omega.normalized()).toRotationMatrix();
}

} // namespace

std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion) {
  auto result = std::make_shared<Scan>();
  result->header = scan.header;
  const std::size_t num_points = scan.points.size();
  if (num_points == 0) {
    return result;
  }

  const auto omega = se3Log(relative_motion);

  result->points.resize(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    const double stamp =
        static_cast<double>(i) / static_cast<double>(num_points - 1);
    const Eigen::Affine3d pose = se3Exp((stamp - 1.0) * omega);

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

  const auto omega_ref = se3Log(relative_motion);

  // Scale twist: relative_motion was observed over delta_t seconds,
  // but this scan spans scan_duration seconds at the known scan_rate.
  const double scan_duration =
      static_cast<double>(num_points - 1) / static_cast<double>(scan_rate);
  const auto omega = omega_ref * (scan_duration / delta_t);

  result->points.resize(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    const double stamp =
        static_cast<double>(i) / static_cast<double>(num_points - 1);
    const Eigen::Affine3d pose = se3Exp((stamp - 1.0) * omega);

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
                             const std::vector<msensor::IMUData> &imu_samples,
                             double delta_t,
                             std::shared_ptr<spdlog::logger> logger) {
  const auto start_time = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&start_time] {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start_time)
        .count();
  };

  if (!std::isfinite(delta_t) || delta_t <= 0.0) {
    throw std::invalid_argument("deskew: delta_t must be finite and positive");
  }

  auto result = std::make_shared<Scan>();
  result->header = scan.header;

  const std::size_t num_points = scan.points.size();
  if (num_points == 0) {
    return result;
  }

  const double start_time_ns = static_cast<double>(scan.header.timestamp);
  const double step_ns = delta_t * kNanosecondsPerSecond;
  const double end_time_ns =
      start_time_ns + static_cast<double>(num_points - 1) * step_ns;

  // Keep only the samples that fall inside the scan window and sort them, so
  // the caller can hand over any IMU batch and the input order does not matter.
  std::vector<msensor::IMUData> samples;
  samples.reserve(imu_samples.size());
  for (const auto &sample : imu_samples) {
    const double sample_time_ns = static_cast<double>(sample.header.timestamp);
    if (sample_time_ns + kTimestampToleranceNs >= start_time_ns &&
        sample_time_ns - kTimestampToleranceNs <= end_time_ns) {
      samples.push_back(sample);
    }
  }
  std::sort(samples.begin(), samples.end(),
            [](const msensor::IMUData &lhs, const msensor::IMUData &rhs) {
              return lhs.header.timestamp < rhs.header.timestamp;
            });

  const auto log_summary = [&] {
    if (logger) {
      logger->info(
          "Deskew took {:.3f} ms; {} of {} IMU samples fit the lidar range",
          elapsed_ms(), samples.size(), imu_samples.size());
    }
  };

  if (samples.empty()) {
    if (logger) {
      logger->warn("Deskew: no IMU samples inside the lidar range, leaving the "
                   "scan undistorted");
    }
    result->points = scan.points;
    log_summary();
    return result;
  }

  result->points.resize(num_points);

  // Orientation of the sensor relative to the scan start, built up as we sweep
  // the point times. The angular velocity is held (zero-order hold) until the
  // next sample is reached.
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  double previous_time_ns = start_time_ns;
  std::size_t sample_index = 0;
  Eigen::Vector3d angular_velocity = angularVelocityOf(samples.front());

  for (std::size_t i = 0; i < num_points; ++i) {
    const double point_time_ns =
        start_time_ns + static_cast<double>(i) * step_ns;

    while (sample_index + 1 < samples.size() &&
           static_cast<double>(samples[sample_index + 1].header.timestamp) <=
               point_time_ns) {
      const double boundary_ns =
          static_cast<double>(samples[sample_index + 1].header.timestamp);
      orientation *=
          rotationIncrement(angular_velocity, (boundary_ns - previous_time_ns) /
                                                  kNanosecondsPerSecond);
      previous_time_ns = boundary_ns;
      ++sample_index;
      angular_velocity = angularVelocityOf(samples[sample_index]);
    }

    orientation *=
        rotationIncrement(angular_velocity, (point_time_ns - previous_time_ns) /
                                                kNanosecondsPerSecond);
    previous_time_ns = point_time_ns;

    const auto &point = scan.points[i];
    const Eigen::Vector3d corrected =
        orientation * Eigen::Vector3d(point.x, point.y, point.z);

    result->points[i].x = static_cast<float>(corrected.x());
    result->points[i].y = static_cast<float>(corrected.y());
    result->points[i].z = static_cast<float>(corrected.z());
    result->points[i].intensity = point.intensity;
  }

  log_summary();
  return result;
}

} // namespace mslam
