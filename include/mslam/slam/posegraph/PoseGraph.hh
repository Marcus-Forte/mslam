#pragma once

#include "mslam/common/Points.hh"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace mslam {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Inverse of a rigid transform by transposing its rotation (more exact than
/// the general 4x4 inverse an Affine3d would use).
inline Eigen::Affine3d inverseRigid(const Eigen::Affine3d &pose) {
  return pose.inverse(Eigen::Isometry);
}

/// Information (inverse covariance) of a relative pose measured with the given
/// 1-sigma values, in the tangent order [rho (translation), omega (rotation)]
/// of se3Log.
inline Matrix6d informationFromSigmas(double sigma_translation,
                                      double sigma_rotation) {
  Matrix6d information = Matrix6d::Zero();
  information.topLeftCorner<3, 3>().diagonal().setConstant(
      1.0 / (sigma_translation * sigma_translation));
  information.bottomRightCorner<3, 3>().diagonal().setConstant(
      1.0 / (sigma_rotation * sigma_rotation));
  return information;
}

/// A keyframe: a pose and the mini map built from the scans taken while it was
/// the current keyframe. Append-only in a PoseGraph.
struct Keyframe {
  std::size_t id = 0;
  std::int64_t stamp_ns = 0;  ///< Time of the scan that opened the keyframe.
  std::size_t scan_count = 0; ///< Scans merged into the mini map.
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();      ///< Optimized.
  Eigen::Affine3d pose_odom = Eigen::Affine3d::Identity(); ///< Never changes.
  /// Everything the keyframe saw, thinned, in the keyframe's own frame: what
  /// the loop verification matches. Sealed (immutable) once the next keyframe
  /// starts. Views of different keyframes overlap.
  std::shared_ptr<const PointCloud> view;
  /// Only the voxels no earlier keyframe owned (a voxel belongs to the first
  /// keyframe that observed it), as centroids in the keyframe's own frame.
  /// Everything published (map re-send, keyframe stream, saved map) is this
  /// cloud, so a voxel appears once in the published map.
  std::shared_ptr<const PointCloud> mini_map;
};

enum class EdgeKind { Odometry, Loop };

/// Constraint `T_from^-1 * T_to ~ measured`.
struct PoseEdge {
  EdgeKind kind = EdgeKind::Odometry;
  std::size_t from = 0;
  std::size_t to = 0;
  Eigen::Affine3d measured = Eigen::Affine3d::Identity(); ///< T_from_to
  Matrix6d information = Matrix6d::Identity(); ///< [rho, omega] order
  bool robust = false;     ///< Huber loss in the optimizer (loop edges).
  double verify_rms = 0.0; ///< Loop verification fit (m).
  double verify_matched_fraction = 0.0; ///< Loop verification fit.
};

/// Keyframes and the edges between them. Keyframe 0 is the gauge: the
/// optimizer never moves it.
struct PoseGraph {
  std::vector<Keyframe> keyframes;
  std::vector<PoseEdge> edges;
};

} // namespace mslam
