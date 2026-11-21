#include "mslam/slam/posegraph/PoseGraphOptimizer.hh"

#include "mslam/slam/SE3.hh"

#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <vector>

namespace mslam {

namespace {

constexpr int kBlock = 6;

double huberCost(double mahalanobis, bool robust, double threshold) {
  if (!robust || mahalanobis <= threshold) {
    return 0.5 * mahalanobis * mahalanobis;
  }
  return threshold * (mahalanobis - 0.5 * threshold);
}

// IRLS weight rho'(e) / e.
double huberWeight(double mahalanobis, bool robust, double threshold) {
  if (!robust || mahalanobis <= threshold) {
    return 1.0;
  }
  return threshold / mahalanobis;
}

Vector6d edgeResidual(const PoseGraph &graph, const PoseEdge &edge) {
  const Eigen::Affine3d error = inverseRigid(edge.measured) *
                                (inverseRigid(graph.keyframes[edge.from].pose) *
                                 graph.keyframes[edge.to].pose);
  return se3Log(error);
}

double graphCost(const PoseGraph &graph, double threshold) {
  double cost = 0.0;
  for (const auto &edge : graph.edges) {
    const Vector6d r = edgeResidual(graph, edge);
    cost += huberCost(std::sqrt(std::max(0.0, r.dot(edge.information * r))),
                      edge.robust, threshold);
  }
  return cost;
}

// Taylor series below this angle: the closed forms cancel catastrophically.
constexpr double kSmallAngle = 1e-2;

Eigen::Matrix3d leftJacobianInverseSO3(const Eigen::Vector3d &omega) {
  const double phi = omega.norm();
  const Eigen::Matrix3d K = hat<double>(omega);
  const double c = phi < kSmallAngle
                       ? 1.0 / 12.0 + phi * phi / 720.0
                       : 1.0 / (phi * phi) - (1.0 + std::cos(phi)) /
                                                 (2.0 * phi * std::sin(phi));
  return Eigen::Matrix3d::Identity() - 0.5 * K + c * K * K;
}

// Barfoot, "State Estimation for Robotics", eq. 7.86.
Eigen::Matrix3d couplingQ(const Eigen::Vector3d &rho,
                          const Eigen::Vector3d &omega) {
  const double phi = omega.norm();
  const Eigen::Matrix3d W = hat<double>(omega);
  const Eigen::Matrix3d P = hat<double>(rho);
  double c1;
  double c2;
  double c3;
  if (phi < kSmallAngle) {
    const double p2 = phi * phi;
    c1 = 1.0 / 6.0 - p2 / 120.0;
    c2 = 1.0 / 24.0 - p2 / 720.0;
    c3 = 1.0 / 120.0 - p2 / 2520.0;
  } else {
    const double p3 = phi * phi * phi;
    c1 = (phi - std::sin(phi)) / p3;
    c2 = (phi * phi + 2.0 * std::cos(phi) - 2.0) / (2.0 * p3 * phi);
    c3 = (2.0 * phi - 3.0 * std::sin(phi) + phi * std::cos(phi)) /
         (2.0 * p3 * phi * phi);
  }
  const Eigen::Matrix3d WPW = W * P * W;
  return 0.5 * P + c1 * (W * P + P * W + WPW) +
         c2 * (W * W * P + P * W * W - 3.0 * WPW) + c3 * (WPW * W + W * WPW);
}

} // namespace

Matrix6d adjoint(const Eigen::Affine3d &pose) {
  const Eigen::Matrix3d R = pose.linear();
  const Eigen::Vector3d t = pose.translation();
  Matrix6d ad = Matrix6d::Zero();
  ad.topLeftCorner<3, 3>() = R;
  ad.topRightCorner<3, 3>() = hat<double>(t) * R;
  ad.bottomRightCorner<3, 3>() = R;
  return ad;
}

Matrix6d leftJacobianInverse(const Vector6d &xi) {
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d omega = xi.tail<3>();
  const Eigen::Matrix3d J_inv = leftJacobianInverseSO3(omega);
  Matrix6d result = Matrix6d::Zero();
  result.topLeftCorner<3, 3>() = J_inv;
  result.topRightCorner<3, 3>() = -J_inv * couplingQ(rho, omega) * J_inv;
  result.bottomRightCorner<3, 3>() = J_inv;
  return result;
}

EdgeLinearization linearizeEdge(const Eigen::Affine3d &from,
                                const Eigen::Affine3d &to,
                                const Eigen::Affine3d &measured) {
  EdgeLinearization out;
  out.residual = se3Log(
      Eigen::Affine3d(inverseRigid(measured) * (inverseRigid(from) * to)));
  // Perturbing T_j on the left by exp(d) turns the error E into
  // exp(Ad((T_i Z)^-1) d) * E, and T_i by exp(d) into exp(-Ad(...) d) * E.
  const Matrix6d jacobian =
      leftJacobianInverse(out.residual) *
      adjoint(Eigen::Affine3d(inverseRigid(Eigen::Affine3d(from * measured))));
  out.jacobian_to = jacobian;
  out.jacobian_from = -jacobian;
  return out;
}

double poseGraphCost(const PoseGraph &graph, double huber_threshold) {
  return graphCost(graph, huber_threshold);
}

OptimizerResult optimizePoseGraph(PoseGraph &graph,
                                  const OptimizerOptions &options) {
  OptimizerResult result;
  const std::size_t n = graph.keyframes.size();
  double cost = graphCost(graph, options.huber_threshold);
  result.initial_cost = cost;
  result.final_cost = cost;
  if (n < 2 || graph.edges.empty()) {
    result.converged = true;
    return result;
  }

  const auto dim = static_cast<Eigen::Index>((n - 1) * kBlock);
  double lambda = options.initial_lambda;
  std::vector<Eigen::Triplet<double>> triplets;
  Eigen::SparseMatrix<double> H(dim, dim);
  Eigen::VectorXd b(dim);
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  std::vector<Eigen::Affine3d> backup(n);

  const auto addBlock = [&triplets](std::size_t row, std::size_t col,
                                    const Matrix6d &block) {
    for (int r = 0; r < kBlock; ++r) {
      for (int c = 0; c < kBlock; ++c) {
        triplets.emplace_back(static_cast<int>(row) * kBlock + r,
                              static_cast<int>(col) * kBlock + c, block(r, c));
      }
    }
  };

  for (unsigned iteration = 0; iteration < options.max_iterations;
       ++iteration) {
    // Linearize at the current poses.
    triplets.clear();
    b.setZero();
    for (const auto &edge : graph.edges) {
      const auto lin =
          linearizeEdge(graph.keyframes[edge.from].pose,
                        graph.keyframes[edge.to].pose, edge.measured);
      const double e = std::sqrt(
          std::max(0.0, lin.residual.dot(edge.information * lin.residual)));
      const Matrix6d W = huberWeight(e, edge.robust, options.huber_threshold) *
                         edge.information;
      // Keyframe 0 has no variables.
      if (edge.from > 0) {
        const std::size_t i = edge.from - 1;
        addBlock(i, i, lin.jacobian_from.transpose() * W * lin.jacobian_from);
        b.segment<kBlock>(static_cast<Eigen::Index>(i) * kBlock) +=
            lin.jacobian_from.transpose() * W * lin.residual;
      }
      if (edge.to > 0) {
        const std::size_t j = edge.to - 1;
        addBlock(j, j, lin.jacobian_to.transpose() * W * lin.jacobian_to);
        b.segment<kBlock>(static_cast<Eigen::Index>(j) * kBlock) +=
            lin.jacobian_to.transpose() * W * lin.residual;
      }
      if (edge.from > 0 && edge.to > 0) {
        const std::size_t i = edge.from - 1;
        const std::size_t j = edge.to - 1;
        const Matrix6d Hij =
            lin.jacobian_from.transpose() * W * lin.jacobian_to;
        addBlock(i, j, Hij);
        addBlock(j, i, Hij.transpose());
      }
    }
    H.setFromTriplets(triplets.begin(), triplets.end());
    const Eigen::VectorXd diagonal = H.diagonal();

    // Levenberg-Marquardt: raise the damping until the cost goes down.
    bool accepted = false;
    double step_norm = 0.0;
    for (int attempt = 0; attempt < 12 && !accepted; ++attempt) {
      Eigen::SparseMatrix<double> damped = H;
      for (Eigen::Index k = 0; k < dim; ++k) {
        damped.coeffRef(k, k) += lambda * std::max(diagonal(k), 1e-9);
      }
      solver.compute(damped);
      if (solver.info() != Eigen::Success) {
        lambda *= 10.0;
        continue;
      }
      const Eigen::VectorXd delta = solver.solve(-b);
      if (solver.info() != Eigen::Success || !delta.allFinite()) {
        lambda *= 10.0;
        continue;
      }

      for (std::size_t k = 1; k < n; ++k) {
        backup[k] = graph.keyframes[k].pose;
        const Vector6d d =
            delta.segment<kBlock>(static_cast<Eigen::Index>(k - 1) * kBlock);
        graph.keyframes[k].pose = se3Exp(d) * graph.keyframes[k].pose;
      }
      const double new_cost = graphCost(graph, options.huber_threshold);
      if (new_cost < cost) {
        step_norm = delta.lpNorm<Eigen::Infinity>();
        result.final_cost = new_cost;
        const double decrease = cost - new_cost;
        cost = new_cost;
        lambda = std::max(lambda / 10.0, 1e-12);
        accepted = true;
        if (decrease <= 1e-14 * std::max(1.0, cost)) {
          step_norm = 0.0; // no measurable progress left
        }
      } else {
        for (std::size_t k = 1; k < n; ++k) {
          graph.keyframes[k].pose = backup[k];
        }
        lambda *= 10.0;
      }
    }

    result.iterations = iteration + 1;
    if (!accepted || step_norm < 1e-10) {
      // No step lowers the cost any more: at a minimum to working precision.
      result.converged = true;
      break;
    }
  }
  return result;
}

} // namespace mslam

namespace mslam {

EdgeResidual edgeResidualNorms(const PoseGraph &graph, const PoseEdge &edge) {
  const Vector6d r = edgeResidual(graph, edge);
  return {r.head<3>().norm(), r.tail<3>().norm()};
}

namespace {

void accumulate(EdgeGroupStats &stats, const EdgeResidual &residual,
                double &sum_t2, double &sum_r2) {
  ++stats.count;
  stats.max_translation_m =
      std::max(stats.max_translation_m, residual.translation_m);
  stats.max_rotation_rad =
      std::max(stats.max_rotation_rad, residual.rotation_rad);
  sum_t2 += residual.translation_m * residual.translation_m;
  sum_r2 += residual.rotation_rad * residual.rotation_rad;
}

void finish(EdgeGroupStats &stats, double sum_t2, double sum_r2) {
  if (stats.count > 0) {
    const auto n = static_cast<double>(stats.count);
    stats.rms_translation_m = std::sqrt(sum_t2 / n);
    stats.rms_rotation_rad = std::sqrt(sum_r2 / n);
  }
}

} // namespace

GraphErrorStats graphErrorStats(const PoseGraph &graph) {
  GraphErrorStats stats;
  double odom_t2 = 0.0, odom_r2 = 0.0, loop_t2 = 0.0, loop_r2 = 0.0;
  for (std::size_t k = 0; k < graph.edges.size(); ++k) {
    const PoseEdge &edge = graph.edges[k];
    const Vector6d r = edgeResidual(graph, edge);
    const EdgeResidual residual{r.head<3>().norm(), r.tail<3>().norm()};
    if (edge.kind == EdgeKind::Loop) {
      accumulate(stats.loop, residual, loop_t2, loop_r2);
    } else {
      accumulate(stats.odometry, residual, odom_t2, odom_r2);
    }
    const double sigmas = std::sqrt(std::max(0.0, r.dot(edge.information * r)));
    if (stats.worst_edge == GraphErrorStats::kNoEdge ||
        sigmas > stats.worst_sigmas) {
      stats.worst_edge = k;
      stats.worst_sigmas = sigmas;
    }
  }
  finish(stats.odometry, odom_t2, odom_r2);
  finish(stats.loop, loop_t2, loop_r2);
  return stats;
}

double normalizedChiSquared(double cost, std::size_t edges) {
  return edges == 0 ? 0.0 : 2.0 * cost / (6.0 * static_cast<double>(edges));
}

PoseDelta poseDelta(const Eigen::Affine3d &from, const Eigen::Affine3d &to) {
  const Eigen::Vector3d moved = to.translation() - from.translation();
  const Eigen::Matrix3d relative = from.linear().transpose() * to.linear();
  return {moved.norm(), Eigen::AngleAxisd(relative).angle()};
}

LargestPoseChange largestPoseChange(const std::vector<Eigen::Affine3d> &before,
                                    const PoseGraph &after) {
  LargestPoseChange change;
  const std::size_t count = std::min(before.size(), after.keyframes.size());
  for (std::size_t k = 0; k < count; ++k) {
    const PoseDelta delta = poseDelta(before[k], after.keyframes[k].pose);
    if (delta.translation_m > change.max.translation_m) {
      change.max.translation_m = delta.translation_m;
      change.translation_keyframe = k;
    }
    if (delta.rotation_rad > change.max.rotation_rad) {
      change.max.rotation_rad = delta.rotation_rad;
      change.rotation_keyframe = k;
    }
  }
  return change;
}

} // namespace mslam
