#pragma once

#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"
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
    transform_ = se3Exp(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(x));
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
 * @brief 3D point-to-plane distance model with an analytical se(3) jacobian.
 *
 * The residual is the signed distance of the transformed source point to the
 * plane through the target point:
 *
 *     d(xi) = n . (q - T(xi) p),   T(xi) = se3Exp(xi),   xi = [rho, omega]
 *
 * which equals `normal * d` of the former 3-vector residual in norm
 * (|normal * d|^2 = d^2 for a unit normal), so the cost and the normal
 * equations are identical for a third of the accumulation work.
 *
 * Input layout: [source_xyz, normal_xyz]  (input_dim = 6)
 * Observation:  [normal . target_xyz]     (observation_dim = 1)
 *
 * The increment is applied on the left of the already transformed source, as
 * in se3Exp(): T p = R(omega) p + V(omega) rho with
 *     R = I + a K + b K^2,  V = I + b K + c K^2,  K = hat(omega),
 *     a = sin(t)/t, b = (1 - cos t)/t^2, c = (1 - a)/t^2, t = |omega|.
 * The jacobian is exact for any xi (not only xi = 0); everything that depends
 * on xi alone is computed once in setState().
 */
struct Point3PlaneDistance {
  void setState(const double *x) {
    rho_ = Eigen::Map<const Eigen::Vector3d>(x);
    omega_ = Eigen::Map<const Eigen::Vector3d>(x + 3);
    const double theta2 = omega_.squaredNorm();
    const double theta = std::sqrt(theta2);

    // a, b, c and (da, db, dc) = (a', b', c') / theta, with Taylor series near
    // zero where the closed forms cancel.
    if (theta < 0.05) {
      const double t4 = theta2 * theta2;
      const double t6 = t4 * theta2;
      a_ = 1.0 - theta2 / 6.0 + t4 / 120.0 - t6 / 5040.0;
      b_ = 0.5 - theta2 / 24.0 + t4 / 720.0 - t6 / 40320.0;
      c_ = 1.0 / 6.0 - theta2 / 120.0 + t4 / 5040.0 - t6 / 362880.0;
      da_ = -1.0 / 3.0 + theta2 / 30.0 - t4 / 840.0;
      db_ = -1.0 / 12.0 + theta2 / 180.0 - t4 / 6720.0;
      dc_ = -1.0 / 60.0 + theta2 / 1260.0 - t4 / 60480.0;
    } else {
      const double sin_t = std::sin(theta);
      const double cos_t = std::cos(theta);
      const double half_sin = std::sin(0.5 * theta);
      a_ = sin_t / theta;
      b_ = 2.0 * half_sin * half_sin / theta2;
      c_ = (1.0 - a_) / theta2;
      da_ = (theta * cos_t - sin_t) / (theta2 * theta);
      db_ = (theta * sin_t - 2.0 * (1.0 - cos_t)) / (theta2 * theta2);
      dc_ = -da_ / theta2 - 2.0 * (1.0 - a_) / (theta2 * theta2);
    }

    const Eigen::Matrix3d k = hat(omega_);
    const Eigen::Matrix3d k2 = k * k;
    rotation_ = Eigen::Matrix3d::Identity() + a_ * k + b_ * k2;
    const Eigen::Matrix3d v = Eigen::Matrix3d::Identity() + b_ * k + c_ * k2;
    translation_ = v * rho_;
    v_transposed_ = v.transpose();

    omega_cross_rho_ = omega_.cross(rho_);
    omega_cross_omega_cross_rho_ = omega_.cross(omega_cross_rho_);
    rho_term_ = b_ * rho_ + c_ * omega_cross_rho_;
    omega_dot_rho_ = omega_.dot(rho_);
  }

  void residual(const double * /*x*/, const double *input,
                const double *plane_offset, double *f_x) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<const Eigen::Vector3d> normal{input + 3};
    f_x[0] = plane_offset[0] - normal.dot(rotation_ * source + translation_);
  }

  /// Writes d(residual)/d[rho, omega] (6 values; active_param_dim = 6).
  void jacobian(const double * /*x*/, const double *input,
                const double * /*plane_offset*/, double *jacobian) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<const Eigen::Vector3d> normal{input + 3};
    Eigen::Map<Eigen::Matrix<double, 6, 1>> gradient{jacobian};

    const Eigen::Vector3d omega_cross_source = omega_.cross(source);
    const double n_dot_ks = normal.dot(omega_cross_source);
    const double scale = da_ * n_dot_ks +
                         db_ * (normal.dot(omega_.cross(omega_cross_source)) +
                                normal.dot(omega_cross_rho_)) +
                         dc_ * normal.dot(omega_cross_omega_cross_rho_);

    // t = a p + b K p + b rho + c K rho, z = b p + c rho
    const Eigen::Vector3d t = a_ * source + b_ * omega_cross_source + rho_term_;
    const double z_dot_omega = b_ * source.dot(omega_) + c_ * omega_dot_rho_;
    const double z_dot_normal = b_ * source.dot(normal) + c_ * rho_.dot(normal);

    gradient.head<3>() = -(v_transposed_ * normal);
    gradient.tail<3>() = -(scale * omega_ + t.cross(normal) +
                           z_dot_omega * normal - z_dot_normal * omega_);
  }

  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();

private:
  Eigen::Vector3d rho_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d v_transposed_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d omega_cross_rho_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_cross_omega_cross_rho_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rho_term_ = Eigen::Vector3d::Zero();
  double omega_dot_rho_ = 0.0;
  double a_ = 1.0, b_ = 0.5, c_ = 1.0 / 6.0;
  double da_ = 0.0, db_ = 0.0, dc_ = 0.0;
};
} // namespace mslam
