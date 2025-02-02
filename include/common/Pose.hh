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

} // namespace mslam