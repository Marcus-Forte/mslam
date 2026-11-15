#pragma once

#include "mslam/slam/SE3.hh"
#include "mslam/slam/SE3Manifold.hh"
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
 * @brief State and conventions shared by the 3D point models.
 *
 * The optimizer variable is the absolute pose T (the first 12 entries of x,
 * SE3Manifold storage). The inputs are source points that were ALREADY moved
 * by `source_pose` (the pose of the current registration round), so the model
 * applies the relative transform T * source_pose^-1 to them. With the default
 * identity `source_pose` the inputs are the raw points. The relative transform
 * is computed once per evaluation point, in setState().
 */
struct SE3PointState {
  SE3PointState() = default;
  explicit SE3PointState(const Eigen::Affine3d &source_pose)
      : source_pose_inverse_(source_pose.inverse()) {}

  void setState(const double *x) {
    const Eigen::Affine3d relative = poseFromStorage(x) * source_pose_inverse_;
    rotation_ = relative.linear();
    translation_ = relative.translation();
  }

  /// T * source_pose^-1 of the last setState().
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();

private:
  Eigen::Affine3d source_pose_inverse_ = Eigen::Affine3d::Identity();
};

/**
 * @brief 3D point-to-point distance model on SE(3).
 *
 * Input: [source_xyz] (input_dim = 3), observation: [target_xyz]
 * (observation_dim = 3). The residual is target - q with q = (T *
 * source_pose^-1) p.
 *
 * The jacobian is with respect to the left increment of SE3Manifold at zero:
 * dq = rho + omega x q, so d r / d rho = -I and d r / d omega = hat(q). It is
 * written transposed (6 x 3, column-major), as moptim expects.
 */
struct Point3Distance : SE3PointState {
  Point3Distance() = default;
  explicit Point3Distance(const Eigen::Affine3d &source_pose)
      : SE3PointState(source_pose) {}

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector3d> target{measurement};
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<Eigen::Vector3d> r{f_x};

    r = target - (rotation_ * source + translation_);
  }

  void jacobian(const double * /*x*/, const double *input,
                const double * /*measurement*/, double *jacobian) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<Eigen::Matrix<double, 6, 3>> jacobian_transposed{jacobian};

    const Eigen::Vector3d q = rotation_ * source + translation_;
    jacobian_transposed.topRows<3>() = -Eigen::Matrix3d::Identity();
    jacobian_transposed.bottomRows<3>() = hat(q).transpose();
  }
};

/**
 * @brief 3D point-to-plane distance model on SE(3), with an analytical
 * jacobian.
 *
 * The residual is the signed distance of the transformed source point to the
 * plane through the target point:
 *
 *     r = d - n . q,   q = (T * source_pose^-1) p
 *
 * (a scalar instead of the 3-vector `n * r`: the cost and the normal equations
 * are the same for a third of the accumulation work).
 *
 * Input layout: [source_xyz, normal_xyz]  (input_dim = 6)
 * Observation:  [normal . target_xyz]     (observation_dim = 1)
 *
 * The state is the pose itself and the optimizer steps it with SE3Manifold, so
 * only the jacobian with respect to the left increment at zero is needed:
 * dq = rho + omega x q gives d r / d rho = -n and d r / d omega = -(q x n).
 */
struct Point3PlaneDistance : SE3PointState {
  Point3PlaneDistance() = default;
  explicit Point3PlaneDistance(const Eigen::Affine3d &source_pose)
      : SE3PointState(source_pose) {}

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

    const Eigen::Vector3d q = rotation_ * source + translation_;
    gradient.head<3>() = -normal;
    gradient.tail<3>() = -q.cross(normal);
  }
};

} // namespace mslam
