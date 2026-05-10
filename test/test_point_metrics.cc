#include "../src/slam/PointDistance.hh"

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace {

template <typename Model, int InputDim, int ObservationDim, int ParamDim>
Eigen::Matrix<double, ParamDim, ObservationDim> finiteDifferenceJacobian(
    Model model, const Eigen::Matrix<double, ParamDim, 1> &params,
    const Eigen::Matrix<double, InputDim, 1> &input,
    const Eigen::Matrix<double, ObservationDim, 1> &measurement) {
  constexpr double kStep = 1e-7;

  Eigen::Matrix<double, ObservationDim, 1> residual;
  Eigen::Matrix<double, ObservationDim, 1> residual_plus;
  Eigen::Matrix<double, ParamDim, ObservationDim> jacobian_transposed;

  Eigen::Matrix<double, ParamDim, 1> params_plus = params;
  model.setState(params.data());
  model.residual(params.data(), input.data(), measurement.data(),
                 residual.data());

  for (int i = 0; i < ParamDim; ++i) {
    params_plus = params;
    params_plus[i] += kStep;
    model.setState(params_plus.data());
    model.residual(params_plus.data(), input.data(), measurement.data(),
                   residual_plus.data());
    jacobian_transposed.row(i) =
        ((residual_plus - residual) / kStep).transpose();
  }

  return jacobian_transposed;
}

TEST(PointMetrics, Point2DistanceAnalyticalJacobianMatchesFiniteDifference) {
  mslam::Point2Distance metric;
  Eigen::Vector3d params(0.15, -0.08, 0.21);
  Eigen::Vector2d input(0.7, -0.4);
  Eigen::Vector2d measurement(1.2, 0.3);

  Eigen::Matrix<double, 3, 2> analytical;
  metric.setState(params.data());
  metric.jacobian(params.data(), input.data(), measurement.data(),
                  analytical.data());

  const auto numerical =
      finiteDifferenceJacobian<mslam::Point2Distance, 2, 2, 3>(
          metric, params, input, measurement);

  EXPECT_TRUE(analytical.isApprox(numerical, 1e-5));
}

TEST(PointMetrics, Point3DistanceAnalyticalJacobianMatchesFiniteDifference) {
  mslam::Point3Distance metric;
  Eigen::Matrix<double, 6, 1> params;
  params << 0.05, -0.03, 0.08, 0.1, -0.07, 0.04;
  Eigen::Vector3d input(0.9, -0.2, 0.5);
  Eigen::Vector3d measurement(1.1, -0.1, 0.4);

  Eigen::Matrix<double, 6, 3> analytical;
  metric.setState(params.data());
  metric.jacobian(params.data(), input.data(), measurement.data(),
                  analytical.data());

  const auto numerical =
      finiteDifferenceJacobian<mslam::Point3Distance, 3, 3, 6>(
          metric, params, input, measurement);

  EXPECT_TRUE(analytical.isApprox(numerical, 1e-5));
}

TEST(PointMetrics,
     Point3PlaneDistanceAnalyticalJacobianMatchesFiniteDifference) {
  mslam::Point3PlaneDistance metric;
  Eigen::Matrix<double, 6, 1> params;
  params << -0.04, 0.06, 0.03, -0.08, 0.05, 0.09;
  Eigen::Matrix<double, 6, 1> input;
  input << 0.2, -0.7, 0.4, 0.0, 0.0, 1.0;
  Eigen::Vector3d measurement(0.3, -0.8, 0.1);

  Eigen::Matrix<double, 6, 3> analytical;
  metric.setState(params.data());
  metric.jacobian(params.data(), input.data(), measurement.data(),
                  analytical.data());

  const auto numerical =
      finiteDifferenceJacobian<mslam::Point3PlaneDistance, 6, 3, 6>(
          metric, params, input, measurement);

  EXPECT_TRUE(analytical.isApprox(numerical, 1e-5));
}

} // namespace