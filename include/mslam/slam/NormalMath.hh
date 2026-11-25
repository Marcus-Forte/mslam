#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cstddef>
#include <limits>

namespace mslam {

/// Normal of the best-fit plane through @p count points (PCA: the eigenvector
/// of the smallest eigenvalue of their covariance), into @p normal. Returns
/// false if there are fewer than 3 points or they are degenerate.
/// `point(i)` returns the i-th point as something with x, y, z (floats).
///
/// The one implementation of NormalEstimator::fromNeighbors().
template <class PointAt>
inline bool normalFromPoints(std::size_t count, PointAt point,
                             Eigen::Vector3d &normal) {
  if (count < 3) {
    return false;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    const auto &p = point(i);
    centroid += Eigen::Vector3d{p.x, p.y, p.z};
  }
  centroid /= static_cast<double>(count);

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    const auto &p = point(i);
    const Eigen::Vector3d centered = Eigen::Vector3d{p.x, p.y, p.z} - centroid;
    covariance.noalias() += centered * centered.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(covariance, Eigen::ComputeEigenvectors);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Vector3d first = solver.eigenvectors().col(0);
  const double norm = first.norm();
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  normal = first / norm;
  return true;
}

} // namespace mslam
