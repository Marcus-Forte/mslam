#include "mslam/slam/SE3.hh"
#include "mslam/slam/registration/PointDistance.hh"

#include "moptim/AnalyticalCost.hh"
#include "moptim/NumericalCostCentral.hh"

#include <gtest/gtest.h>

#include <array>
#include <random>
#include <vector>

namespace {

using mslam::Point3PlaneDistance;
using Vector6 = Eigen::Matrix<double, 6, 1>;

// The former residual: normal * (normal . (target - T source)), a 3-vector.
Eigen::Vector3d legacyResidual(const Vector6 &xi, const Eigen::Vector3d &source,
                               const Eigen::Vector3d &normal,
                               const Eigen::Vector3d &target) {
  const Eigen::Affine3d transform = mslam::se3Exp(xi);
  return normal * normal.dot(target - transform * source);
}

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

std::array<double, 6> inputOf(const Sample &s) {
  return {s.source.x(), s.source.y(), s.source.z(),
          s.normal.x(), s.normal.y(), s.normal.z()};
}

std::vector<Vector6> testIncrements() {
  std::vector<Vector6> xs;
  Vector6 x;
  x.setZero();
  xs.push_back(x);
  x << 1e-9, -2e-9, 3e-9, 1e-9, -1e-9, 2e-9; // below the Taylor threshold
  xs.push_back(x);
  x << 0.01, -0.02, 0.03, 0.001, -0.002, 0.003;
  xs.push_back(x);
  x << 0.1, -0.2, 0.3, 0.02, 0.04, -0.03; // theta ~ 0.054, just above 0.05
  xs.push_back(x);
  x << -0.5, 0.25, 0.75, 0.3, -0.2, 0.1;
  xs.push_back(x);
  x << 1.0, 2.0, -1.5, -0.7, 0.9, 1.1;
  xs.push_back(x);
  x << 0.3, 0.1, -0.2, 2.5, -1.0, 0.5; // large rotation (|omega| ~ 2.7)
  xs.push_back(x);
  return xs;
}

TEST(Point3PlaneDistance, ResidualMatchesLegacyThreeVectorNorm) {
  for (const auto &xi : testIncrements()) {
    Point3PlaneDistance model;
    model.setState(xi.data());
    for (const auto &sample : makeSamples(20, 7)) {
      const auto input = inputOf(sample);
      const double offset = sample.normal.dot(sample.target);
      double scalar = 0.0;
      model.residual(xi.data(), input.data(), &offset, &scalar);

      const Eigen::Vector3d legacy =
          legacyResidual(xi, sample.source, sample.normal, sample.target);
      EXPECT_NEAR(scalar * scalar, legacy.squaredNorm(), 1e-9);
      // The scalar is exactly the signed distance: legacy = normal * scalar.
      EXPECT_NEAR((legacy - sample.normal * scalar).norm(), 0.0, 1e-9);
    }
  }
}

TEST(Point3PlaneDistance, JacobianMatchesCentralDifferences) {
  constexpr double h = 1e-6;
  for (const auto &xi : testIncrements()) {
    for (const auto &sample : makeSamples(20, 11)) {
      const auto input = inputOf(sample);
      const double offset = sample.normal.dot(sample.target);

      Point3PlaneDistance model;
      model.setState(xi.data());
      Vector6 analytic;
      model.jacobian(xi.data(), input.data(), &offset, analytic.data());

      Vector6 numeric;
      for (int k = 0; k < 6; ++k) {
        Vector6 plus = xi;
        Vector6 minus = xi;
        plus[k] += h;
        minus[k] -= h;
        Point3PlaneDistance model_plus;
        Point3PlaneDistance model_minus;
        model_plus.setState(plus.data());
        model_minus.setState(minus.data());
        double r_plus = 0.0;
        double r_minus = 0.0;
        model_plus.residual(plus.data(), input.data(), &offset, &r_plus);
        model_minus.residual(minus.data(), input.data(), &offset, &r_minus);
        numeric[k] = (r_plus - r_minus) / (2.0 * h);
      }
      EXPECT_NEAR((analytic - numeric).norm(), 0.0, 1e-6)
          << "xi = " << xi.transpose();
    }
  }
}

TEST(Point3PlaneDistance, ResidualMatchesSe3Exp) {
  for (const auto &xi : testIncrements()) {
    Point3PlaneDistance model;
    model.setState(xi.data());
    const Eigen::Affine3d reference = mslam::se3Exp(xi);
    EXPECT_NEAR((model.rotation_ - reference.linear()).norm(), 0.0, 1e-9);
    EXPECT_NEAR((model.translation_ - reference.translation()).norm(), 0.0,
                1e-9);
  }
}

// The analytical cost, evaluated through moptim, produces the same normal
// equations as the numerical cost of the same scalar model.
TEST(Point3PlaneDistance, AnalyticalCostMatchesNumericalNormalEquations) {
  const auto samples = makeSamples(300, 3);
  std::vector<double> inputs;
  std::vector<double> offsets;
  for (const auto &sample : samples) {
    const auto input = inputOf(sample);
    inputs.insert(inputs.end(), input.begin(), input.end());
    offsets.push_back(sample.normal.dot(sample.target));
  }

  moptim::AnalyticalCost<Point3PlaneDistance, double> analytical(
      inputs.data(), offsets.data(), samples.size(), 6, 1, 6);
  moptim::NumericalCostCentral<Point3PlaneDistance, double> numerical(
      inputs.data(), offsets.data(), samples.size(), 6, 1, 6);

  for (const auto &xi : testIncrements()) {
    Eigen::Matrix<double, 6, 6> jtj_a;
    Eigen::Matrix<double, 6, 6> jtj_n;
    Vector6 jtb_a;
    Vector6 jtb_n;
    double cost_a = 0.0;
    double cost_n = 0.0;
    analytical.computeLinearSystem(xi.data(), jtj_a.data(), jtb_a.data(),
                                   cost_a);
    numerical.computeLinearSystem(xi.data(), jtj_n.data(), jtb_n.data(),
                                  cost_n);
    EXPECT_NEAR(cost_a, cost_n, 1e-9 * (1.0 + cost_n));
    EXPECT_NEAR((jtj_a - jtj_n).norm() / (1.0 + jtj_n.norm()), 0.0, 1e-6);
    EXPECT_NEAR((jtb_a - jtb_n).norm() / (1.0 + jtb_n.norm()), 0.0, 1e-6);

    // And the cost equals the sum of squared legacy 3-vector residuals.
    double legacy_cost = 0.0;
    for (const auto &sample : samples) {
      legacy_cost +=
          legacyResidual(xi, sample.source, sample.normal, sample.target)
              .squaredNorm();
    }
    EXPECT_NEAR(cost_a, legacy_cost, 1e-8 * (1.0 + legacy_cost));
  }
}

} // namespace
