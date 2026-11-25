#include "mslam/slam/SE3.hh"
#include "mslam/slam/SE3Manifold.hh"
#include "mslam/slam/registration/PointDistance.hh"

#include "moptim/AnalyticalCost.hh"
#include "moptim/NumericalCostCentral.hh"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <random>
#include <vector>

namespace {

using mslam::Point3Distance;
using mslam::Point3PlaneDistance;
using mslam::SE3Manifold;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Pose = Eigen::Matrix<double, 12, 1>;

struct Sample {
  Eigen::Vector3d source;
  Eigen::Vector3d normal;
  Eigen::Vector3d target;
};

std::vector<Sample> makeSamples(int count, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
  std::normal_distribution<double> gaussian(0.0, 1.0);
  std::vector<Sample> samples;
  for (int i = 0; i < count; ++i) {
    Sample sample;
    sample.source = {coordinate(rng), coordinate(rng), coordinate(rng)};
    sample.normal = Eigen::Vector3d(gaussian(rng), gaussian(rng), gaussian(rng))
                        .normalized();
    sample.target =
        sample.source +
        0.2 * Eigen::Vector3d(gaussian(rng), gaussian(rng), gaussian(rng));
    samples.push_back(sample);
  }
  return samples;
}

std::array<double, 6> planeInputOf(const Sample &s) {
  return {s.source.x(), s.source.y(), s.source.z(),
          s.normal.x(), s.normal.y(), s.normal.z()};
}

Pose storageOf(const Eigen::Affine3d &pose) {
  Pose x;
  mslam::poseToStorage(pose, x.data());
  return x;
}

// Absolute poses, including large rotations.
std::vector<Eigen::Affine3d> testPoses() {
  std::vector<Vector6> twists;
  Vector6 x;
  x.setZero();
  twists.push_back(x);
  x << 1e-9, -2e-9, 3e-9, 1e-9, -1e-9, 2e-9;
  twists.push_back(x);
  x << 0.01, -0.02, 0.03, 0.001, -0.002, 0.003;
  twists.push_back(x);
  x << -0.5, 0.25, 0.75, 0.3, -0.2, 0.1;
  twists.push_back(x);
  x << 1.0, 2.0, -1.5, -0.7, 0.9, 1.1;
  twists.push_back(x);
  x << 0.3, 0.1, -0.2, 2.5, -1.0, 0.5; // |omega| ~ 2.7
  twists.push_back(x);
  x << -3.0, 4.0, 0.5, -1.2, 2.2, -2.0; // |omega| ~ 3.1
  twists.push_back(x);

  std::vector<Eigen::Affine3d> poses;
  for (const auto &twist : twists) {
    poses.push_back(mslam::se3Exp(twist));
  }
  return poses;
}

// A pose that is neither the identity nor small, used as the pose the inputs
// were already moved by.
Eigen::Affine3d someSourcePose() {
  Vector6 twist;
  twist << 0.7, -0.4, 1.3, 0.4, -0.9, 1.7;
  return mslam::se3Exp(twist);
}

// Central differences of the residual through the manifold's plus.
template <class Model>
Eigen::MatrixXd numericalJacobian(Model model, const Pose &x,
                                  const double *input, const double *obs,
                                  int residual_dim) {
  constexpr double h = 1e-6;
  const SE3Manifold manifold;
  Eigen::MatrixXd jac(residual_dim, 6);
  for (int k = 0; k < 6; ++k) {
    Vector6 step = Vector6::Zero();
    step[k] = h;
    Pose plus;
    manifold.plus(x.data(), step.data(), plus.data());
    step[k] = -h;
    Pose minus;
    manifold.plus(x.data(), step.data(), minus.data());

    Eigen::VectorXd r_plus(residual_dim);
    Eigen::VectorXd r_minus(residual_dim);
    model.setState(plus.data());
    model.residual(plus.data(), input, obs, r_plus.data());
    model.setState(minus.data());
    model.residual(minus.data(), input, obs, r_minus.data());
    jac.col(k) = (r_plus - r_minus) / (2.0 * h);
  }
  return jac;
}

// ---- Point3PlaneDistance ----

TEST(Point3PlaneDistance, ResidualIsSignedDistanceToThePlane) {
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Point3PlaneDistance model;
    model.setState(x.data());
    for (const auto &sample : makeSamples(20, 7)) {
      const auto input = planeInputOf(sample);
      const double offset = sample.normal.dot(sample.target);
      double residual = 0.0;
      model.residual(x.data(), input.data(), &offset, &residual);
      EXPECT_NEAR(residual,
                  sample.normal.dot(sample.target - pose * sample.source),
                  1e-9);
    }
  }
}

TEST(Point3PlaneDistance, JacobianMatchesCentralDifferencesThroughPlus) {
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    for (const auto &sample : makeSamples(20, 11)) {
      const auto input = planeInputOf(sample);
      const double offset = sample.normal.dot(sample.target);

      Point3PlaneDistance model;
      model.setState(x.data());
      Vector6 analytic;
      model.jacobian(x.data(), input.data(), &offset, analytic.data());

      const Eigen::MatrixXd numeric =
          numericalJacobian(Point3PlaneDistance{}, x, input.data(), &offset, 1);
      EXPECT_NEAR((analytic - numeric.transpose()).norm(), 0.0, 1e-6);
    }
  }
}

// Point3PlaneDistance is written as scalar code on AVX+FMA builds (the Eigen
// version stalls on masked loads there). The scalar code is meant to round
// like the Eigen expressions it replaced, bit for bit; this checks it against
// those expressions, kept here as the reference.
TEST(Point3PlaneDistance, ScalarCodeRoundsLikeTheEigenExpressions) {
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Point3PlaneDistance model;
    model.setState(x.data());
    for (const auto &sample : makeSamples(2000, 23)) {
      const auto input = planeInputOf(sample);
      const double offset = sample.normal.dot(sample.target);

      Eigen::Map<const Eigen::Vector3d> source{input.data()};
      Eigen::Map<const Eigen::Vector3d> normal{input.data() + 3};
      const Eigen::Vector3d q = model.rotation_ * source + model.translation_;
      const double expected_residual = offset - normal.dot(q);
      Vector6 expected_jacobian;
      expected_jacobian.head<3>() = -normal;
      expected_jacobian.tail<3>() = -q.cross(normal);

      double residual = 0.0;
      model.residual(x.data(), input.data(), &offset, &residual);
      Vector6 jacobian;
      model.jacobian(x.data(), input.data(), &offset, jacobian.data());

      EXPECT_EQ(residual, expected_residual);
      for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(jacobian[i], expected_jacobian[i]) << "entry " << i;
      }
    }
  }
}

// The analytical cost, evaluated through moptim, produces the same normal
// equations as the numerical cost of the same scalar model on the same
// manifold.
TEST(Point3PlaneDistance, AnalyticalCostMatchesNumericalNormalEquations) {
  const auto samples = makeSamples(300, 3);
  std::vector<double> inputs;
  std::vector<double> offsets;
  for (const auto &sample : samples) {
    const auto input = planeInputOf(sample);
    inputs.insert(inputs.end(), input.begin(), input.end());
    offsets.push_back(sample.normal.dot(sample.target));
  }

  const auto manifold = std::make_shared<const SE3Manifold>();
  moptim::AnalyticalCost<Point3PlaneDistance, double> analytical(
      inputs.data(), offsets.data(), samples.size(), 6, 1, manifold,
      Point3PlaneDistance{}, 6);
  moptim::NumericalCostCentral<Point3PlaneDistance, double> numerical(
      inputs.data(), offsets.data(), samples.size(), 6, 1, manifold);

  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Eigen::Matrix<double, 6, 6> jtj_a;
    Eigen::Matrix<double, 6, 6> jtj_n;
    Vector6 jtb_a;
    Vector6 jtb_n;
    double cost_a = 0.0;
    double cost_n = 0.0;
    analytical.computeLinearSystem(x.data(), jtj_a.data(), jtb_a.data(),
                                   cost_a);
    numerical.computeLinearSystem(x.data(), jtj_n.data(), jtb_n.data(), cost_n);
    EXPECT_NEAR(cost_a, cost_n, 1e-9 * (1.0 + cost_n));
    EXPECT_NEAR((jtj_a - jtj_n).norm() / (1.0 + jtj_n.norm()), 0.0, 1e-6);
    EXPECT_NEAR((jtb_a - jtb_n).norm() / (1.0 + jtb_n.norm()), 0.0, 1e-6);

    // And the cost equals the sum of squared distances to the planes.
    double expected_cost = 0.0;
    for (const auto &sample : samples) {
      const double d = sample.normal.dot(sample.target - pose * sample.source);
      expected_cost += d * d;
    }
    EXPECT_NEAR(cost_a, expected_cost, 1e-8 * (1.0 + expected_cost));
  }
}

// Inputs already moved by source_pose, with the model told that pose, give the
// residuals and jacobian of the unmoved inputs.
TEST(Point3PlaneDistance, SourcePoseCompensatesMovedInputs) {
  const Eigen::Affine3d source_pose = someSourcePose();
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Point3PlaneDistance reference;
    reference.setState(x.data());
    Point3PlaneDistance moved(source_pose);
    moved.setState(x.data());
    for (const auto &sample : makeSamples(10, 5)) {
      const auto input = planeInputOf(sample);
      Sample moved_sample = sample;
      moved_sample.source = source_pose * sample.source;
      const auto moved_input = planeInputOf(moved_sample);
      const double offset = sample.normal.dot(sample.target);

      double r_ref = 0.0;
      double r_moved = 0.0;
      reference.residual(x.data(), input.data(), &offset, &r_ref);
      moved.residual(x.data(), moved_input.data(), &offset, &r_moved);
      EXPECT_NEAR(r_ref, r_moved, 1e-9);

      Vector6 j_ref;
      Vector6 j_moved;
      reference.jacobian(x.data(), input.data(), &offset, j_ref.data());
      moved.jacobian(x.data(), moved_input.data(), &offset, j_moved.data());
      EXPECT_NEAR((j_ref - j_moved).norm(), 0.0, 1e-9);
    }
  }
}

// ---- Point3Distance ----

TEST(Point3Distance, ResidualIsTargetMinusTransformedSource) {
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Point3Distance model;
    model.setState(x.data());
    for (const auto &sample : makeSamples(20, 9)) {
      Eigen::Vector3d residual;
      model.residual(x.data(), sample.source.data(), sample.target.data(),
                     residual.data());
      EXPECT_NEAR((residual - (sample.target - pose * sample.source)).norm(),
                  0.0, 1e-9);
    }
  }
}

TEST(Point3Distance, JacobianMatchesCentralDifferencesThroughPlus) {
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    for (const auto &sample : makeSamples(20, 13)) {
      Point3Distance model;
      model.setState(x.data());
      // Transposed: 6 x 3, column-major.
      Eigen::Matrix<double, 6, 3> analytic;
      model.jacobian(x.data(), sample.source.data(), sample.target.data(),
                     analytic.data());

      const Eigen::MatrixXd numeric = numericalJacobian(
          Point3Distance{}, x, sample.source.data(), sample.target.data(), 3);
      EXPECT_NEAR((analytic - numeric.transpose()).norm(), 0.0, 1e-6);
    }
  }
}

TEST(Point3Distance, AnalyticalCostMatchesNumericalNormalEquations) {
  const auto samples = makeSamples(200, 4);
  std::vector<double> inputs;
  std::vector<double> targets;
  for (const auto &sample : samples) {
    inputs.insert(inputs.end(), sample.source.data(), sample.source.data() + 3);
    targets.insert(targets.end(), sample.target.data(),
                   sample.target.data() + 3);
  }

  const auto manifold = std::make_shared<const SE3Manifold>();
  moptim::AnalyticalCost<Point3Distance, double> analytical(
      inputs.data(), targets.data(), samples.size(), 3, 3, manifold,
      Point3Distance{}, 6);
  moptim::NumericalCostCentral<Point3Distance, double> numerical(
      inputs.data(), targets.data(), samples.size(), 3, 3, manifold);

  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Eigen::Matrix<double, 6, 6> jtj_a;
    Eigen::Matrix<double, 6, 6> jtj_n;
    Vector6 jtb_a;
    Vector6 jtb_n;
    double cost_a = 0.0;
    double cost_n = 0.0;
    analytical.computeLinearSystem(x.data(), jtj_a.data(), jtb_a.data(),
                                   cost_a);
    numerical.computeLinearSystem(x.data(), jtj_n.data(), jtb_n.data(), cost_n);
    EXPECT_NEAR(cost_a, cost_n, 1e-9 * (1.0 + cost_n));
    EXPECT_NEAR((jtj_a - jtj_n).norm() / (1.0 + jtj_n.norm()), 0.0, 1e-6);
    EXPECT_NEAR((jtb_a - jtb_n).norm() / (1.0 + jtb_n.norm()), 0.0, 1e-6);
  }
}

TEST(Point3Distance, SourcePoseCompensatesMovedInputs) {
  const Eigen::Affine3d source_pose = someSourcePose();
  for (const auto &pose : testPoses()) {
    const Pose x = storageOf(pose);
    Point3Distance reference;
    reference.setState(x.data());
    Point3Distance moved(source_pose);
    moved.setState(x.data());
    for (const auto &sample : makeSamples(10, 6)) {
      const Eigen::Vector3d moved_source = source_pose * sample.source;

      Eigen::Vector3d r_ref;
      Eigen::Vector3d r_moved;
      reference.residual(x.data(), sample.source.data(), sample.target.data(),
                         r_ref.data());
      moved.residual(x.data(), moved_source.data(), sample.target.data(),
                     r_moved.data());
      EXPECT_NEAR((r_ref - r_moved).norm(), 0.0, 1e-9);

      Eigen::Matrix<double, 6, 3> j_ref;
      Eigen::Matrix<double, 6, 3> j_moved;
      reference.jacobian(x.data(), sample.source.data(), sample.target.data(),
                         j_ref.data());
      moved.jacobian(x.data(), moved_source.data(), sample.target.data(),
                     j_moved.data());
      EXPECT_NEAR((j_ref - j_moved).norm(), 0.0, 1e-9);
    }
  }
}

} // namespace
