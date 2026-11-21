#include "mslam/slam/SE3.hh"
#include "mslam/slam/posegraph/PoseGraphOptimizer.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace {

using mslam::Matrix6d;
using mslam::PoseEdge;
using mslam::PoseGraph;
using mslam::Vector6d;

Eigen::Affine3d expPose(double x, double y, double z, double rx, double ry,
                        double rz) {
  Vector6d xi;
  xi << x, y, z, rx, ry, rz;
  return mslam::se3Exp(xi);
}

Vector6d residual(const Eigen::Affine3d &from, const Eigen::Affine3d &to,
                  const Eigen::Affine3d &measured) {
  return mslam::linearizeEdge(from, to, measured).residual;
}

// Central finite differences of the residual with respect to LEFT increments.
void finiteDifferences(const Eigen::Affine3d &from, const Eigen::Affine3d &to,
                       const Eigen::Affine3d &measured, Matrix6d &d_from,
                       Matrix6d &d_to) {
  constexpr double h = 1e-6;
  for (int k = 0; k < 6; ++k) {
    Vector6d step = Vector6d::Zero();
    step(k) = h;
    const Eigen::Affine3d plus = mslam::se3Exp(step);
    const Eigen::Affine3d minus = mslam::se3Exp(Vector6d(-step));
    d_to.col(k) = (residual(from, plus * to, measured) -
                   residual(from, minus * to, measured)) /
                  (2 * h);
    d_from.col(k) = (residual(plus * from, to, measured) -
                     residual(minus * from, to, measured)) /
                    (2 * h);
  }
}

double jacobianError(const Eigen::Affine3d &from, const Eigen::Affine3d &to,
                     const Eigen::Affine3d &measured) {
  const auto lin = mslam::linearizeEdge(from, to, measured);
  Matrix6d fd_from;
  Matrix6d fd_to;
  finiteDifferences(from, to, measured, fd_from, fd_to);
  return std::max((lin.jacobian_from - fd_from).cwiseAbs().maxCoeff(),
                  (lin.jacobian_to - fd_to).cwiseAbs().maxCoeff());
}

TEST(PoseGraphJacobian, MatchesFiniteDifferencesAtZeroResidual) {
  const auto from = expPose(1.0, -2.0, 0.5, 0.3, -0.2, 1.1);
  const auto to = expPose(1.4, -1.5, 0.6, 0.35, -0.1, 1.3);
  const Eigen::Affine3d measured = from.inverse() * to; // exact: r = 0
  EXPECT_LT(mslam::linearizeEdge(from, to, measured).residual.norm(), 1e-12);
  EXPECT_LT(jacobianError(from, to, measured), 1e-7);
}

TEST(PoseGraphJacobian, MatchesFiniteDifferencesAtARealisticResidual) {
  const auto from = expPose(1.0, -2.0, 0.5, 0.3, -0.2, 1.1);
  const auto to = expPose(1.4, -1.5, 0.6, 0.35, -0.1, 1.3);
  // About 0.3 m and 0.25 rad of disagreement.
  const Eigen::Affine3d measured =
      from.inverse() * to * expPose(0.2, -0.15, 0.1, 0.12, -0.1, 0.18);
  const auto lin = mslam::linearizeEdge(from, to, measured);
  EXPECT_GT(lin.residual.tail<3>().norm(), 0.2);
  EXPECT_LT(jacobianError(from, to, measured), 1e-7);

  // Dropping Jl^-1 (identity) would not be good enough here.
  const Matrix6d without =
      mslam::adjoint(Eigen::Affine3d((from * measured).inverse()));
  EXPECT_GT((lin.jacobian_to - without).cwiseAbs().maxCoeff(), 1e-2);
}

TEST(PoseGraphJacobian, MatchesFiniteDifferencesForSmallAndLargeRotations) {
  const auto from = expPose(0.2, 0.1, -0.3, -0.4, 0.5, 0.2);
  const auto to = expPose(-0.5, 0.7, 0.2, 0.1, 0.3, -0.6);
  // Rotation residual angle 5e-3 (series branch of Jl^-1) and 1.5 rad.
  for (const double angle : {5e-3, 1e-4, 0.8, 1.5}) {
    const Eigen::Affine3d measured =
        from.inverse() * to *
        expPose(0.05, -0.02, 0.03, angle * 0.6, angle * 0.64, angle * 0.48);
    EXPECT_LT(jacobianError(from, to, measured), 1e-6) << angle;
  }
}

// ---------------------------------------------------------------------------
// A square of keyframes with noisy odometry.

struct Square {
  std::vector<Eigen::Affine3d> truth;
  PoseGraph graph;
};

Square makeSquare(unsigned seed, double side = 4.0, int per_side = 16) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  Square square;
  const double step = side / per_side;
  const int n = 4 * per_side;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  for (int k = 0; k < n; ++k) {
    const int leg = k / per_side;
    const double yaw = leg * M_PI / 2.0;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = position;
    square.truth.push_back(pose);
    position += step * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  }

  for (int k = 0; k < n; ++k) {
    mslam::Keyframe keyframe;
    keyframe.id = static_cast<std::size_t>(k);
    square.graph.keyframes.push_back(keyframe);
  }
  square.graph.keyframes[0].pose = square.truth[0];
  const Matrix6d information = mslam::informationFromSigmas(0.01, 0.005);
  for (int k = 0; k + 1 < n; ++k) {
    Vector6d noise;
    noise << 0.01 * normal(rng), 0.01 * normal(rng), 0.01 * normal(rng),
        0.005 * normal(rng), 0.005 * normal(rng), 0.005 * normal(rng);
    PoseEdge edge;
    edge.from = static_cast<std::size_t>(k);
    edge.to = static_cast<std::size_t>(k + 1);
    edge.measured = square.truth[static_cast<std::size_t>(k)].inverse() *
                    square.truth[static_cast<std::size_t>(k + 1)] *
                    mslam::se3Exp(noise);
    edge.information = information;
    square.graph.edges.push_back(edge);
    square.graph.keyframes[edge.to].pose =
        square.graph.keyframes[edge.from].pose * edge.measured;
  }
  return square;
}

PoseEdge loopEdge(const Square &square, std::size_t from, std::size_t to,
                  double sigma_t, double sigma_r, bool robust,
                  const Eigen::Affine3d &error = Eigen::Affine3d::Identity()) {
  PoseEdge edge;
  edge.kind = mslam::EdgeKind::Loop;
  edge.from = from;
  edge.to = to;
  edge.measured = square.truth[from].inverse() * square.truth[to] * error;
  edge.information = mslam::informationFromSigmas(sigma_t, sigma_r);
  edge.robust = robust;
  return edge;
}

double positionRms(const PoseGraph &a, const PoseGraph &b) {
  double sum = 0.0;
  for (std::size_t k = 0; k < a.keyframes.size(); ++k) {
    sum +=
        (a.keyframes[k].pose.translation() - b.keyframes[k].pose.translation())
            .squaredNorm();
  }
  return std::sqrt(sum / static_cast<double>(a.keyframes.size()));
}

double positionRmsTruth(const PoseGraph &graph,
                        const std::vector<Eigen::Affine3d> &truth) {
  double sum = 0.0;
  for (std::size_t k = 0; k < truth.size(); ++k) {
    sum += (graph.keyframes[k].pose.translation() - truth[k].translation())
               .squaredNorm();
  }
  return std::sqrt(sum / static_cast<double>(truth.size()));
}

TEST(PoseGraphOptimizer, SquareLoopClosesBelowAMillimetre) {
  Square square = makeSquare(7);
  const std::size_t last = square.truth.size() - 1;
  square.graph.edges.push_back(loopEdge(square, 0, last, 1e-4, 1e-5, false));

  const double drift_before = positionRmsTruth(square.graph, square.truth);
  const Eigen::Affine3d gauge = square.graph.keyframes[0].pose;
  const auto result = mslam::optimizePoseGraph(square.graph);

  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.final_cost, result.initial_cost);
  const Vector6d closing = residual(square.graph.keyframes[0].pose,
                                    square.graph.keyframes[last].pose,
                                    square.graph.edges.back().measured);
  EXPECT_LT(closing.head<3>().norm(), 1e-3);
  EXPECT_LT(closing.tail<3>().norm(), 1e-4);
  // Closing the loop also pulls the chain towards the truth.
  EXPECT_GT(drift_before, 0.02);
  EXPECT_LT(positionRmsTruth(square.graph, square.truth), drift_before);
  // The gauge is untouched, bit for bit.
  EXPECT_TRUE(square.graph.keyframes[0].pose.matrix() == gauge.matrix());
}

TEST(PoseGraphOptimizer, WithoutALoopTheOdometryChainIsAlreadyAMinimum) {
  Square square = makeSquare(3);
  const auto before = square.graph;
  const auto result = mslam::optimizePoseGraph(square.graph);
  EXPECT_TRUE(result.converged);
  EXPECT_LT(positionRms(before, square.graph), 1e-9);
}

TEST(PoseGraphOptimizer, HuberLimitsTheInfluenceOfAFalseLoop) {
  // Three good loop closures across the square and one false one, 3 m off.
  const auto build = [](bool with_bad, bool robust_bad) {
    Square square = makeSquare(11);
    const std::size_t n = square.truth.size();
    for (const std::size_t from : {std::size_t{4}, std::size_t{20}}) {
      square.graph.edges.push_back(
          loopEdge(square, from, from + n / 2 - 1, 0.02, 0.01, true));
    }
    square.graph.edges.push_back(loopEdge(square, 10, n - 3, 0.02, 0.01, true));
    if (with_bad) {
      square.graph.edges.push_back(
          loopEdge(square, 8, 40, 0.02, 0.01, robust_bad,
                   expPose(3.0, -2.0, 0.0, 0.0, 0.0, 0.3)));
    }
    mslam::optimizePoseGraph(square.graph);
    return square.graph;
  };
  const PoseGraph clean = build(false, false);
  const PoseGraph robust = build(true, true);
  const PoseGraph plain = build(true, false);

  const double robust_shift = positionRms(robust, clean);
  const double plain_shift = positionRms(plain, clean);
  EXPECT_GT(plain_shift, 0.2);
  EXPECT_LT(robust_shift, plain_shift / 3.0);
  EXPECT_LT(robust_shift, 0.1);
}

TEST(PoseGraphOptimizer, GaugeStaysFixedWithManyEdges) {
  Square square = makeSquare(5);
  const std::size_t n = square.truth.size();
  square.graph.edges.push_back(loopEdge(square, 0, n - 1, 0.02, 0.01, true));
  square.graph.edges.push_back(loopEdge(square, 1, n - 2, 0.02, 0.01, true));
  // Start the graph away from the measurements, so every free pose moves.
  for (std::size_t k = 1; k < n; ++k) {
    square.graph.keyframes[k].pose =
        expPose(0.1, -0.1, 0.05, 0.02, 0.01, -0.03) *
        square.graph.keyframes[k].pose;
  }
  const Eigen::Affine3d gauge = square.graph.keyframes[0].pose;
  mslam::optimizePoseGraph(square.graph);
  EXPECT_TRUE(square.graph.keyframes[0].pose.matrix() == gauge.matrix());
}

// ---------------------------------------------------------------------------
// Error statistics (what the optimization log line reports).

Eigen::Affine3d translation(double x, double y = 0.0, double z = 0.0) {
  return Eigen::Affine3d(Eigen::Translation3d(x, y, z));
}

Eigen::Affine3d yaw(double angle) {
  return Eigen::Affine3d(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

// Three keyframes and three edges with known residuals:
//   e0 odometry 0->1: 3 mm translation, no rotation
//   e1 odometry 1->2: no translation, 4 mrad rotation
//   e2 loop     0->2: 4 mm translation, no rotation (sigma 1 mm: the worst)
PoseGraph knownResidualsGraph() {
  PoseGraph graph;
  graph.keyframes.resize(3);
  graph.keyframes[1].pose = translation(1.0);
  graph.keyframes[2].pose = translation(2.0) * yaw(0.004);

  PoseEdge e0;
  e0.from = 0;
  e0.to = 1;
  e0.measured = translation(1.003);
  e0.information = mslam::informationFromSigmas(0.01, 0.01);
  PoseEdge e1 = e0;
  e1.from = 1;
  e1.to = 2;
  e1.measured = translation(1.0);
  PoseEdge e2;
  e2.kind = mslam::EdgeKind::Loop;
  e2.from = 0;
  e2.to = 2;
  e2.measured = translation(2.004) * yaw(0.004);
  e2.information = mslam::informationFromSigmas(0.001, 0.001);
  graph.edges = {e0, e1, e2};
  return graph;
}

TEST(PoseGraphErrorStats, EdgeResidualNormsSplitTranslationAndRotation) {
  const PoseGraph graph = knownResidualsGraph();
  const auto e0 = mslam::edgeResidualNorms(graph, graph.edges[0]);
  EXPECT_NEAR(e0.translation_m, 3e-3, 1e-12);
  EXPECT_NEAR(e0.rotation_rad, 0.0, 1e-12);
  const auto e1 = mslam::edgeResidualNorms(graph, graph.edges[1]);
  EXPECT_NEAR(e1.translation_m, 0.0, 1e-12);
  EXPECT_NEAR(e1.rotation_rad, 4e-3, 1e-12);
  const auto e2 = mslam::edgeResidualNorms(graph, graph.edges[2]);
  EXPECT_NEAR(e2.translation_m, 4e-3, 1e-12);
  EXPECT_NEAR(e2.rotation_rad, 0.0, 1e-12);
}

TEST(PoseGraphErrorStats, GroupsMaxRmsAndTheWorstEdge) {
  const PoseGraph graph = knownResidualsGraph();
  const auto stats = mslam::graphErrorStats(graph);

  EXPECT_EQ(stats.odometry.count, 2U);
  EXPECT_NEAR(stats.odometry.max_translation_m, 3e-3, 1e-12);
  EXPECT_NEAR(stats.odometry.rms_translation_m, std::sqrt(9e-6 / 2.0), 1e-12);
  EXPECT_NEAR(stats.odometry.max_rotation_rad, 4e-3, 1e-12);
  EXPECT_NEAR(stats.odometry.rms_rotation_rad, std::sqrt(16e-6 / 2.0), 1e-12);

  EXPECT_EQ(stats.loop.count, 1U);
  EXPECT_NEAR(stats.loop.max_translation_m, 4e-3, 1e-12);
  EXPECT_NEAR(stats.loop.rms_translation_m, 4e-3, 1e-12);
  EXPECT_NEAR(stats.loop.max_rotation_rad, 0.0, 1e-12);

  // In sigmas: e0 0.3, e1 0.4, e2 4.0 (4 mm over a 1 mm sigma).
  EXPECT_EQ(stats.worst_edge, 2U);
  EXPECT_NEAR(stats.worst_sigmas, 4.0, 1e-9);
}

TEST(PoseGraphErrorStats, EmptyGraphAndTheNormalizedCost) {
  const auto stats = mslam::graphErrorStats(PoseGraph{});
  EXPECT_EQ(stats.worst_edge, mslam::GraphErrorStats::kNoEdge);
  EXPECT_EQ(stats.odometry.count + stats.loop.count, 0U);

  const PoseGraph graph = knownResidualsGraph();
  const double cost = mslam::poseGraphCost(graph);
  // 0.5 * (0.3^2 + 0.4^2 + 4^2) in sigmas.
  EXPECT_NEAR(cost, 0.5 * (0.09 + 0.16 + 16.0), 1e-9);
  EXPECT_NEAR(mslam::normalizedChiSquared(cost, graph.edges.size()),
              2.0 * cost / (6.0 * 3.0), 1e-12);
  EXPECT_EQ(mslam::normalizedChiSquared(1.0, 0), 0.0);
}

TEST(PoseGraphErrorStats, PoseDeltaAndTheLargestKeyframeChange) {
  const auto delta = mslam::poseDelta(
      translation(1.0), translation(1.0, 0.003, 0.004) * yaw(0.002));
  EXPECT_NEAR(delta.translation_m, 5e-3, 1e-12);
  EXPECT_NEAR(delta.rotation_rad, 2e-3, 1e-12);

  PoseGraph graph = knownResidualsGraph();
  std::vector<Eigen::Affine3d> before;
  for (const auto &kf : graph.keyframes) {
    before.push_back(kf.pose);
  }
  // Keyframe 1 moves 2 mm, keyframe 2 moves 6 mm and turns 3 mrad.
  graph.keyframes[1].pose = translation(1.002);
  graph.keyframes[2].pose = translation(2.006) * yaw(0.004 + 0.003);
  const auto change = mslam::largestPoseChange(before, graph);
  EXPECT_EQ(change.translation_keyframe, 2U);
  EXPECT_NEAR(change.max.translation_m, 6e-3, 1e-12);
  EXPECT_EQ(change.rotation_keyframe, 2U);
  EXPECT_NEAR(change.max.rotation_rad, 3e-3, 1e-12);
}

} // namespace
