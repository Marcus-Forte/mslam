#pragma once

#include "mslam/slam/SE3.hh"
#include "mslam/slam/SE3Manifold.hh"
#include "mslam/slam/Transform.hh"
#include <Eigen/Dense>

#include <cmath>

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

#if defined(__AVX__) && defined(__FMA__)
/// Keeps the compiler from fusing the value into a following add (FMA
/// contraction): an empty asm that "modifies" it.
inline double opaque(double value) {
  asm("" : "+x"(value));
  return value;
}

struct PlanePoint {
  double x, y, z;
};

/// R * p + t in the operation order of Eigen's AVX code: per row, a plain
/// product with column 0, then two fused multiply-adds with columns 1 and 2,
/// then a plain add of the translation. `state.rotation_` is column-major.
template <class State>
inline PlanePoint transformPoint(const State &state, const double *p) {
  const double *r = state.rotation_.data();
  const double *t = state.translation_.data();
  const double x = std::fma(r[6], p[2], std::fma(r[3], p[1], r[0] * p[0]));
  const double y = std::fma(r[7], p[2], std::fma(r[4], p[1], r[1] * p[0]));
  const double z = std::fma(r[8], p[2], std::fma(r[5], p[1], r[2] * p[0]));
  return {x + t[0], y + t[1], z + t[2]};
}
#endif

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

#if defined(__AVX__) && defined(__FMA__)
  // Scalar version for AVX+FMA builds. The Eigen version below compiles, with
  // -mavx, to masked loads/stores (vmaskmovpd) for its size-3 vectors, which
  // cannot be store-forwarded: it cost 10.9 + 25.7 ns per element in the
  // residual and the jacobian, 2.6 times more than the same code without AVX
  // (bench_slam solve 675 ms -> 271 ms with this version, 2026-11-25).
  //
  // The result is bit-identical to the Eigen version: the sequence of
  // operations below, which ones are fused and which are rounded, is the one
  // of that version's machine code on this target, read from the assembly
  // (Eigen's packet code, with GCC's FMA contraction): the plain products
  // that are summed are kept from being fused by opaque(), and every fusion
  // is written as std::fma, so the order does not depend on the compiler's
  // contraction. Do not "simplify" the expressions; the pose of a recording
  // depends on their last bit. Other targets (SSE only, NEON) keep the Eigen
  // code, whose rounding they already have.
  void residual(const double * /*x*/, const double *input,
                const double *plane_offset, double *f_x) const {
    const detail::PlanePoint q = detail::transformPoint(*this, input);
    const double n0 = input[3], n1 = input[4], n2 = input[5];
    const double sum = detail::opaque(n0 * q.x) + detail::opaque(n1 * q.y);
    f_x[0] = plane_offset[0] - std::fma(n2, q.z, sum);
  }

  /// Writes d(residual)/d[rho, omega] (6 values; active_param_dim = 6).
  void jacobian(const double * /*x*/, const double *input,
                const double * /*plane_offset*/, double *jacobian) const {
    const detail::PlanePoint q = detail::transformPoint(*this, input);
    const double n0 = input[3], n1 = input[4], n2 = input[5];
    jacobian[0] = -n0;
    jacobian[1] = -n1;
    jacobian[2] = -n2;
    // -(q x n); each component is one rounded product and one fused one.
    jacobian[3] = -std::fma(-q.z, n1, n2 * q.y);
    jacobian[4] = -std::fma(-q.x, n2, n0 * q.z);
    jacobian[5] = -std::fma(q.x, n1, -(n0 * q.y));
  }
#else
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
#endif
};

} // namespace mslam
