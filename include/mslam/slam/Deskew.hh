#pragma once

#include "msensor/interface/IImu.hh"
#include "mslam/common/Points.hh"

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace spdlog {
class logger;
} // namespace spdlog

namespace mslam {

/// Deskew a scan by applying the inverse of a relative motion observed over the
/// scan, using a constant-velocity twist.
std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion);

/// Time-aware deskew: scales the twist from relative_motion (observed over
/// delta_t seconds) to match the actual scan duration derived from scan_rate.
std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion,
                             unsigned int scan_rate, double delta_t);

/**
 * @brief Deskew a LiDAR scan using IMU gyroscope samples.
 *
 * LiDAR points are assumed to be sampled at a constant interval: point i is
 * acquired at `scan.header.timestamp + i * delta_t`. The IMU samples provide
 * the body-frame angular velocity during the scan; they are integrated
 * (zero-order hold) to estimate the sensor orientation at every point's
 * acquisition time. Each point is then rotated into the scan-start frame, so
 * the first point is returned unchanged and later points have their rotational
 * distortion removed.
 *
 * The samples are filtered to the scan window
 * `[scan_start, scan_start + (num_points - 1) * delta_t]`: samples outside are
 * ignored. Samples are sorted internally, so the input order does not matter.
 * The number of samples that fit the window and the time spent deskewing are
 * logged when a logger is provided.
 *
 * Only rotation is compensated. Accelerometer dead-reckoning over a single scan
 * is too noisy to recover translation reliably, so the translational part of
 * the motion is left untouched.
 *
 * @param scan        Single LiDAR scan. `header.timestamp` is the acquisition
 *                    time of `points[0]`, in nanoseconds.
 * @param imu_samples Candidate IMU measurements; those inside the scan window
 *                    are used.
 * @param delta_t     Constant time between consecutive LiDAR points, in
 *                    seconds.
 * @param logger      Optional logger for the timing / sample-count diagnostics.
 * @return A new scan in the scan-start frame. The header and per-point
 *         intensities are preserved. If no IMU sample falls inside the scan
 *         window, the points are returned unchanged.
 *
 * @throws std::invalid_argument if @p delta_t is not finite and positive. An
 *         empty scan returns an empty result without validating the IMU input.
 */
std::shared_ptr<Scan> deskew(const Scan &scan,
                             const std::vector<msensor::IMUData> &imu_samples,
                             double delta_t,
                             std::shared_ptr<spdlog::logger> logger = nullptr);

} // namespace mslam
