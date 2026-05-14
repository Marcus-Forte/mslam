#pragma once

#include "slam/Transform.hh"
#include <Eigen/Dense>

namespace mslam {
namespace detail {

inline Eigen::Matrix2d rotation2D(double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  Eigen::Matrix2d rotation;
  rotation << c, -s, s, c;
  return rotation;
}

inline Eigen::Matrix2d rotation2DDerivative(double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  Eigen::Matrix2d derivative;
  derivative << -s, -c, c, -s;
  return derivative;
}

struct Rotation3DTerms {
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d d_roll;
  Eigen::Matrix3d d_pitch;
  Eigen::Matrix3d d_yaw;
};

inline Rotation3DTerms rotation3DTerms(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Eigen::Matrix3d rx;
  rx << 1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr;
  Eigen::Matrix3d ry;
  ry << cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp;
  Eigen::Matrix3d rz;
  rz << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix3d drx;
  drx << 0.0, 0.0, 0.0, 0.0, -sr, -cr, 0.0, cr, -sr;
  Eigen::Matrix3d dry;
  dry << -sp, 0.0, cp, 0.0, 0.0, 0.0, -cp, 0.0, -sp;
  Eigen::Matrix3d drz;
  drz << -sy, -cy, 0.0, cy, -sy, 0.0, 0.0, 0.0, 0.0;

  return {
      rx * ry * rz,
      drx * ry * rz,
      rx * dry * rz,
      rx * ry * drz,
  };
}

} // namespace detail

/**
 * @brief 2D Point distance model
 *
 */
struct Point2Distance {
  void setState(const double *x) { transform_ = toAffine(x[0], x[1], x[2]); }

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector2d> target{measurement};
    Eigen::Map<const Eigen::Vector2d> source{input};
    Eigen::Map<Eigen::Vector2d> transformed_point{f_x};

    transformed_point = target - transform_ * source;
  }

  void jacobian(const double *x, const double *input,
                const double * /*measurement*/, double *jacobian) const {
    Eigen::Map<const Eigen::Vector2d> source{input};
    Eigen::Map<Eigen::Matrix<double, 3, 2>> jacobian_transposed{jacobian};

    const Eigen::Matrix2d rotation_derivative =
        detail::rotation2DDerivative(x[2]);
    const Eigen::Vector2d d_theta = -rotation_derivative * source;

    jacobian_transposed.col(0) << -1.0, 0.0, d_theta.x();
    jacobian_transposed.col(1) << 0.0, -1.0, d_theta.y();
  }

  Eigen::Affine2d transform_;
};

/**
 * @brief 3D Point distance model
 *
 */
struct Point3Distance {
  void setState(const double *x) {
    transform_ = toAffine(x[0], x[1], x[2], x[3], x[4], x[5]);
  }

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector3d> target{measurement};
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<Eigen::Vector3d> transformed_point{f_x};

    transformed_point = target - transform_ * source;
  }

  void jacobian(const double *x, const double *input,
                const double * /*measurement*/, double *jacobian) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<Eigen::Matrix<double, 6, 3>> jacobian_transposed{jacobian};

    const auto rotation_terms = detail::rotation3DTerms(x[3], x[4], x[5]);
    const Eigen::Vector3d d_roll = -rotation_terms.d_roll * source;
    const Eigen::Vector3d d_pitch = -rotation_terms.d_pitch * source;
    const Eigen::Vector3d d_yaw = -rotation_terms.d_yaw * source;

    jacobian_transposed.col(0) << -1.0, 0.0, 0.0, d_roll.x(), d_pitch.x(),
        d_yaw.x();
    jacobian_transposed.col(1) << 0.0, -1.0, 0.0, d_roll.y(), d_pitch.y(),
        d_yaw.y();
    jacobian_transposed.col(2) << 0.0, 0.0, -1.0, d_roll.z(), d_pitch.z(),
        d_yaw.z();
  }

  Eigen::Affine3d transform_;
};

/**
 * @brief 3D point-to-plane distance model
 *
 * Input layout: [source_xyz, normal_xyz]
 * Measurement layout: [target_xyz]
 */
struct Point3PlaneDistance {
  void setState(const double *x) {
    transform_ = toAffine(x[0], x[1], x[2], x[3], x[4], x[5]);
  }

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<const Eigen::Vector3d> normal{input + 3};
    Eigen::Map<const Eigen::Vector3d> target{measurement};
    Eigen::Map<Eigen::Vector3d> residual_vector{f_x};

    const double signed_distance = normal.dot(target - transform_ * source);
    residual_vector = normal * signed_distance;
  }

  void jacobian(const double *x, const double *input, const double *measurement,
                double *jacobian) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<const Eigen::Vector3d> normal{input + 3};
    Eigen::Map<const Eigen::Vector3d> /*target*/ target{measurement};
    Eigen::Map<Eigen::Matrix<double, 6, 3>> jacobian_transposed{jacobian};

    const auto rotation_terms = detail::rotation3DTerms(x[3], x[4], x[5]);
    const Eigen::Vector3d d_roll = -rotation_terms.d_roll * source;
    const Eigen::Vector3d d_pitch = -rotation_terms.d_pitch * source;
    const Eigen::Vector3d d_yaw = -rotation_terms.d_yaw * source;

    Eigen::Matrix<double, 6, 1> signed_distance_jacobian;
    signed_distance_jacobian << -normal.x(), -normal.y(), -normal.z(),
        normal.dot(d_roll), normal.dot(d_pitch), normal.dot(d_yaw);

    jacobian_transposed.col(0) = signed_distance_jacobian * normal.x();
    jacobian_transposed.col(1) = signed_distance_jacobian * normal.y();
    jacobian_transposed.col(2) = signed_distance_jacobian * normal.z();
  }

  Eigen::Affine3d transform_;
};
} // namespace mslam