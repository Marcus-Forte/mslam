#pragma once

#include "mslam/slam/posegraph/PoseGraph.hh"

namespace mslam {

/// Adjoint of an SE(3) transform in the [rho, omega] tangent order:
/// T * exp(xi) * T^-1 = exp(Ad(T) * xi).
Matrix6d adjoint(const Eigen::Affine3d &pose);

/// Inverse of the left Jacobian of SE(3) at @p xi ([rho, omega] order), so that
/// log(exp(delta) * exp(xi)) ~ xi + Jl^-1(xi) * delta for a small delta.
Matrix6d leftJacobianInverse(const Vector6d &xi);

/// Residual of one edge and its Jacobians with respect to LEFT increments
/// (T <- se3Exp(d) * T, the SE3Manifold convention) of the two poses.
struct EdgeLinearization {
  Vector6d residual;      ///< se3Log(Z^-1 * T_i^-1 * T_j)
  Matrix6d jacobian_from; ///< d residual / d d_i = -jacobian_to
  Matrix6d jacobian_to;   ///< d residual / d d_j = Jl^-1(r) * Ad((T_i Z)^-1)
};

EdgeLinearization linearizeEdge(const Eigen::Affine3d &from,
                                const Eigen::Affine3d &to,
                                const Eigen::Affine3d &measured);

struct OptimizerOptions {
  unsigned max_iterations = 20;
  /// Huber threshold on the Mahalanobis norm sqrt(r' * Omega * r) of a robust
  /// edge; beyond it the cost grows linearly.
  double huber_threshold = 1.345;
  double initial_lambda = 1e-4; ///< Levenberg-Marquardt damping.
};

struct OptimizerResult {
  unsigned iterations = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  bool converged = false;
};

/// Sparse Levenberg-Marquardt over the poses of keyframes 1..N-1 (keyframe 0
/// stays fixed). The cost is the sum over edges of 0.5 * r' * Omega * r, with
/// the Huber loss on edges flagged `robust` (applied by IRLS weights).
/// Deterministic: the result depends only on the graph.
OptimizerResult optimizePoseGraph(PoseGraph &graph,
                                  const OptimizerOptions &options = {});

/// Cost of the graph as the optimizer defines it.
double poseGraphCost(const PoseGraph &graph, double huber_threshold = 1.345);

// ---------------------------------------------------------------------------
// Error statistics in physical units, for reporting. Pure functions of a graph
// (they change nothing); compute the "before" figures before
// optimizePoseGraph() mutates it.

/// Size of one edge's residual `se3Log(Z^-1 * T_i^-1 * T_j)`: the norms of its
/// translation part (m) and of its rotation part (rad).
struct EdgeResidual {
  double translation_m = 0.0;
  double rotation_rad = 0.0;
};
EdgeResidual edgeResidualNorms(const PoseGraph &graph, const PoseEdge &edge);

/// Max and RMS residual of a group of edges.
struct EdgeGroupStats {
  std::size_t count = 0;
  double max_translation_m = 0.0;
  double rms_translation_m = 0.0;
  double max_rotation_rad = 0.0;
  double rms_rotation_rad = 0.0;
};

/// Residuals of all edges, split by kind, and the worst edge: the one with the
/// largest sigma-normalised error sqrt(r' * Omega * r), the only measure that
/// compares an odometry edge with a loop edge.
struct GraphErrorStats {
  EdgeGroupStats odometry;
  EdgeGroupStats loop;
  static constexpr std::size_t kNoEdge = static_cast<std::size_t>(-1);
  std::size_t worst_edge = kNoEdge; ///< index into PoseGraph::edges
  double worst_sigmas = 0.0;        ///< its sqrt(r' * Omega * r)
};
GraphErrorStats graphErrorStats(const PoseGraph &graph);

/// Cost normalised by the degrees of freedom: `2 * cost / (6 * edges)`. About 1
/// when the residuals match their sigmas (0 for a graph without edges).
double normalizedChiSquared(double cost, std::size_t edges);

/// Displacement from one pose to another: how far the origin moved (m) and the
/// angle of the relative rotation (rad).
struct PoseDelta {
  double translation_m = 0.0;
  double rotation_rad = 0.0;
};
PoseDelta poseDelta(const Eigen::Affine3d &from, const Eigen::Affine3d &to);

/// The largest displacement of a keyframe between @p before (one pose per
/// keyframe of @p after, which must have at least that many) and the poses of
/// @p after, with the keyframe that moved most in each measure.
struct LargestPoseChange {
  PoseDelta max;
  std::size_t translation_keyframe = 0;
  std::size_t rotation_keyframe = 0;
};
LargestPoseChange largestPoseChange(const std::vector<Eigen::Affine3d> &before,
                                    const PoseGraph &after);

} // namespace mslam
