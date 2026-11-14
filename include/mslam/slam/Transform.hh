#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace mslam {

Eigen::Affine2d toAffine(double x, double y, double theta);

/// Pose from a translation and the Euler angles (rx, ry, rz) of
/// R = Rx(rx) * Ry(ry) * Rz(rz).
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz);

/// Pose of a SLAM state (position and Euler rotation).
Eigen::Affine3d toAffine(const SlamState &state);

/// Rotation matrix of the Euler angles (rx, ry, rz), as used by toAffine().
Eigen::Matrix3d eulerToRotation(const Eigen::Vector3d &euler);

/// Euler angles (rx, ry, rz) of a rotation matrix; the inverse of
/// eulerToRotation() for ry in [-pi/2, pi/2].
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d &rotation);

/// Writes the position and Euler rotation of @p pose into @p state; velocity
/// and biases are left untouched.
void setPose(SlamState &state, const Eigen::Affine3d &pose);

/// A state holding only @p pose (zero velocity and biases).
SlamState toSlamState(const Eigen::Affine3d &pose);

void transformCloud(const Eigen::Affine3d &transform, PointCloud &cloud_out);

void transformCloud(const Eigen::Affine2d &transform, PointCloud &cloud_out);

} // namespace mslam
