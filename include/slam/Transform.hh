#pragma once

#include "common/Points.hh"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Affine2d toAffine(double x, double y, double theta);
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz);

void transformCloud(const Eigen::Affine3d &transform,
                    mslam::PointCloud &cloud_out);

void transformCloud(const Eigen::Affine2d &transform,
                    mslam::PointCloud &cloud_out);

/// @brief Skew-symmetric matrix from a 3-vector.
inline Eigen::Matrix3d hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

/// @brief SE3 logarithm: Affine3d -> 6-vector (translation part, rotation
/// part).
inline Eigen::Matrix<double, 6, 1> se3Log(const Eigen::Affine3d &T) {
  const Eigen::Matrix3d R = T.rotation();
  const Eigen::Vector3d t = T.translation();

  const Eigen::AngleAxisd aa(R);
  const double theta = aa.angle();
  const Eigen::Vector3d omega = aa.axis() * theta;

  Eigen::Matrix3d V_inv;
  if (theta < 1e-10) {
    V_inv = Eigen::Matrix3d::Identity();
  } else {
    const Eigen::Matrix3d omega_hat = hat(omega);
    V_inv = Eigen::Matrix3d::Identity() - 0.5 * omega_hat +
            (1.0 / (theta * theta)) *
                (1.0 - (theta * std::cos(theta / 2.0)) /
                           (2.0 * std::sin(theta / 2.0))) *
                omega_hat * omega_hat;
  }

  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = V_inv * t;
  xi.tail<3>() = omega;
  return xi;
}

/// @brief SE3 exponential: 6-vector -> Affine3d.
inline Eigen::Affine3d se3Exp(const Eigen::Matrix<double, 6, 1> &xi) {
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d omega = xi.tail<3>();
  const double theta = omega.norm();

  Eigen::Matrix3d R;
  Eigen::Matrix3d V;

  if (theta < 1e-10) {
    R = Eigen::Matrix3d::Identity();
    V = Eigen::Matrix3d::Identity();
  } else {
    const Eigen::Matrix3d omega_hat = hat(omega);
    const Eigen::Matrix3d omega_hat2 = omega_hat * omega_hat;
    const double s = std::sin(theta) / theta;
    const double c = (1.0 - std::cos(theta)) / (theta * theta);

    R = Eigen::Matrix3d::Identity() + s * omega_hat + c * omega_hat2;
    V = Eigen::Matrix3d::Identity() + c * omega_hat +
        ((1.0 - s) / (theta * theta)) * omega_hat2;
  }

  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = R;
  T.translation() = V * rho;
  return T;
}