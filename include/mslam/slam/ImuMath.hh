#pragma once

#include "msensor/interface/IImu.hh"
#include <Eigen/Dense>
#include <optional>

namespace mslam {

inline constexpr double k_gravity_mps2 = 9.80665;

/**
 * @brief Rotate a body-frame accelerometer sample into the world frame and
 *        remove gravity.
 *
 * @param rotation Euler angles (rx, ry, rz) of the body in the world frame.
 * @param acceleration_scale Scale applied to the raw accelerometer reading.
 */
Eigen::Vector3d
toGravityCompensatedWorldAcceleration(const Eigen::Vector3d &rotation,
                                      const msensor::IMUData &imu_data,
                                      double acceleration_scale);

/**
 * @brief Estimate roll and pitch (x, y) from the gravity direction measured
 *        by the accelerometer.
 *
 * @return std::nullopt if the measured acceleration is not finite or too small
 *         to define a direction.
 */
std::optional<Eigen::Vector2d>
estimateGravityAlignedRollPitch(const msensor::IMUData &imu_data);

} // namespace mslam
