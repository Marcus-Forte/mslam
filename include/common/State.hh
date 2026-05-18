#pragma once

#include <Eigen/Dense>

namespace mslam {
/**
 * @brief x, y, theta.
 *
 */
using Pose2D = Eigen::Vector3d;

/**
 * @brief x, y, z, phi, omega, theta.
 *
 */
using Pose3D = Eigen::Matrix<double, 1, 6>;

/**
 * @brief Full SLAM state: 6-DOF pose + velocity + IMU biases.
 */
struct SlamState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();   // x, y, z,
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();   // phi, omega, theta.
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();   // vx, vy, vz
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();  // bg_x, bg_y, bg_z
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero(); // ba_x, ba_y, ba_z
};

} // namespace mslam