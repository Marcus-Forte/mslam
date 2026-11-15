#pragma once

#include "mslam/slam/SE3.hh"

#include "moptim/Manifold.hh"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mslam {

/**
 * @brief The pose manifold SE(3) for the optimizer.
 *
 * Ambient storage (12 doubles): the 3x4 matrix [R | t] in column-major order,
 * so Eigen::Map<const Eigen::Matrix<double, 3, 4>>(x) is the pose, and it is
 * the layout of Eigen::Affine3d::affine().
 *
 * Tangent (6 doubles): [rho, omega], the ordering of se3Exp().
 *
 * plus(T, delta) = se3Exp(delta) * T: a left increment with coupled
 * translation. A model's jacobian with respect to the step at zero is
 * therefore the derivative with respect to a left increment: for a point
 * q = T p, dq = rho + omega x q.
 *
 * The rotation is not re-orthonormalised: each step multiplies by an exact
 * rotation (up to rounding), so the drift stays at the rounding level.
 */
class SE3Manifold final : public moptim::IManifold<double> {
public:
  static constexpr int kAmbientDim = 12;
  static constexpr int kTangentDim = 6;

  size_t ambientDim() const override { return kAmbientDim; }
  size_t tangentDim() const override { return kTangentDim; }

  void plus(const double *x, const double *delta,
            double *x_plus) const override {
    const Eigen::Affine3d step =
        se3Exp(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta));
    const Eigen::Map<const Eigen::Matrix<double, 3, 4>> pose(x);
    Eigen::Map<Eigen::Matrix<double, 3, 4>> result(x_plus);
    result.leftCols<3>() = step.linear() * pose.leftCols<3>();
    result.col(3) = step.linear() * pose.col(3) + step.translation();
  }
};

/// Writes @p pose into the 12 doubles at @p x ([R | t], column-major).
inline void poseToStorage(const Eigen::Affine3d &pose, double *x) {
  Eigen::Map<Eigen::Matrix<double, 3, 4>> storage(x);
  storage = pose.affine();
}

/// The pose stored in the 12 doubles at @p x.
inline Eigen::Affine3d poseFromStorage(const double *x) {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.affine() = Eigen::Map<const Eigen::Matrix<double, 3, 4>>(x);
  return pose;
}

} // namespace mslam
