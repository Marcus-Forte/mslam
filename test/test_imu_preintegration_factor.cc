#include "mslam/slam/ImuPreintegration.hh"
#include "mslam/slam/SE3.hh"
#include "mslam/slam/SE3Manifold.hh"
#include "mslam/slam/registration/ImuPreintegrationFactor.hh"

#include <gtest/gtest.h>

namespace {

using Factor = mslam::ImuPreintegrationFactor;

constexpr double kWeight = 0.37;

mslam::ImuPreintegrator makePreintegrator() {
  mslam::ImuPreintegrator preintegrator{mslam::ImuNoiseParams{}};
  for (int i = 0; i < 40; ++i) {
    const double t = 0.005 * i;
    preintegrator.integrate(
        Eigen::Vector3d(0.1 * std::sin(t), 0.2, -0.05 + 0.1 * t),
        Eigen::Vector3d(0.3, -0.2 * std::cos(t), 9.8), 0.005);
  }
  return preintegrator;
}

struct Fixture {
  mslam::ImuPreintegrator preintegrator = makePreintegrator();
  Eigen::Matrix<double, Factor::kInputDim, 1> input;
  Eigen::Matrix<double, Factor::kAmbientDim, 1> x;

  Fixture() {
    Eigen::Matrix<double, 6, 1> twist_i;
    twist_i << 0.1, -0.2, 0.3, 0.05, -0.1, 0.2;
    Eigen::Matrix<double, 6, 1> twist_j;
    twist_j << 0.15, -0.18, 0.31, 0.06, -0.12, 0.23;
    mslam::poseToStorage(mslam::se3Exp(twist_i), input.data());
    input.tail<9>() << 0.1, 0.2, -0.3, 1e-4, -2e-4, 3e-4, 1e-3, 2e-3, -1e-3;
    mslam::poseToStorage(mslam::se3Exp(twist_j), x.data());
    x.segment<9>(12) << 0.12, 0.19, -0.28, 2e-4, -1e-4, 2e-4, 2e-3, 1e-3, -2e-3;
    x.segment<9>(21) << 0.09, 0.22, -0.31, -1e-4, 3e-4, 5e-4, -1e-3, 3e-3, 2e-3;
  }
};

mslam::ImuState stateOf(const double *pose, const double *extras) {
  const Eigen::Affine3d T = mslam::poseFromStorage(pose);
  mslam::ImuState state;
  state.position = T.translation();
  state.rotation = T.linear();
  state.velocity = Eigen::Map<const Eigen::Vector3d>(extras);
  state.gyro_bias = Eigen::Map<const Eigen::Vector3d>(extras + 3);
  state.accel_bias = Eigen::Map<const Eigen::Vector3d>(extras + 6);
  return state;
}

// Static IMU with white accelerometer noise only (zero gyro, no bias): over
// T = N*dt the velocity error has variance na^2*T and the position error
// na^2*T^3/3, with correlation sqrt(3)/2 ~ 0.87 between them. The
// preintegration covariance must reproduce these to within discretisation.
TEST(ImuPreintegrator, CovarianceOfStaticAccelNoiseMatchesContinuousModel) {
  const mslam::ImuNoiseParams noise;
  mslam::ImuPreintegrator preintegrator{noise};
  constexpr int kSamples = 10;
  constexpr double kDt = 0.0048;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel(0.0, 0.0, 9.80665);
  for (int i = 0; i < kSamples; ++i) {
    preintegrator.integrate(gyro, accel, kDt);
  }

  const double T = kSamples * kDt;
  const double na2 = noise.accel_noise_density * noise.accel_noise_density;
  const auto &cov = preintegrator.covariance();
  const Eigen::Matrix3d cov_vv = cov.block<3, 3>(3, 3);
  const Eigen::Matrix3d cov_pp = cov.block<3, 3>(6, 6);
  const Eigen::Matrix3d cov_vp = cov.block<3, 3>(3, 6);

  const double std_v = std::sqrt(cov_vv.diagonal().mean());
  const double std_p = std::sqrt(cov_pp.diagonal().mean());
  const double expected_std_v = std::sqrt(na2 * T);
  const double expected_std_p = std::sqrt(na2 * T * T * T / 3.0);
  EXPECT_NEAR(std_v / expected_std_v, 1.0, 0.02);
  EXPECT_NEAR(std_p / expected_std_p, 1.0, 0.02);

  const double corr = cov_vp.diagonal().mean() / (std_v * std_p);
  EXPECT_NEAR(corr, std::sqrt(3.0) / 2.0, 0.02);
}

TEST(ImuPreintegrationFactor, Dimensions) {
  EXPECT_EQ(Factor::kAmbientDim, 30);
  EXPECT_EQ(Factor::kTangentDim, 24);
  EXPECT_EQ(Factor::kInputDim, 21);
  EXPECT_EQ(Factor::kResidualDim, 24);
  const auto manifold = Factor::makeManifold();
  EXPECT_EQ(manifold->ambientDim(), 30U);
  EXPECT_EQ(manifold->tangentDim(), 24U);
}

// The residual is the weighted preintegration residual of (state_i, state_j)
// stacked with the weighted prior on state_i's velocity and biases.
TEST(ImuPreintegrationFactor, ResidualIsPreintegrationPlusPrior) {
  const Fixture f;
  const mslam::ImuPriorSigma sigma;
  const Factor factor(f.preintegrator, kWeight, sigma);

  Eigen::Matrix<double, Factor::kResidualDim, 1> res;
  const Eigen::Matrix<double, Factor::kResidualDim, 1> zeros =
      Eigen::Matrix<double, Factor::kResidualDim, 1>::Zero();
  factor.residual(f.x.data(), f.input.data(), zeros.data(), res.data());

  const mslam::ImuState state_j = stateOf(f.x.data(), f.x.data() + 12);
  const mslam::ImuState state_i = stateOf(f.input.data(), f.x.data() + 21);
  const Eigen::Matrix<double, 15, 1> expected_head =
      kWeight * f.preintegrator.sqrtInformation() *
      f.preintegrator.residual(state_i, state_j);
  EXPECT_NEAR((res.head<15>() - expected_head).norm(), 0.0,
              1e-9 * (1.0 + expected_head.norm()));

  Eigen::Matrix<double, 9, 1> sqrt_info;
  sqrt_info << Eigen::Vector3d::Constant(1.0 / sigma.velocity),
      Eigen::Vector3d::Constant(1.0 / sigma.gyro_bias),
      Eigen::Vector3d::Constant(1.0 / sigma.accel_bias);
  const Eigen::Matrix<double, 9, 1> expected_tail =
      kWeight * sqrt_info.cwiseProduct(f.x.tail<9>() - f.input.tail<9>());
  EXPECT_NEAR((res.tail<9>() - expected_tail).norm(), 0.0,
              1e-9 * (1.0 + expected_tail.norm()));
}

// A step through the product manifold moves the pose of state_j on SE(3) and
// adds the rest to the matching entries; the residual follows the right
// blocks.
TEST(ImuPreintegrationFactor, ManifoldStepPerturbsTheRightBlocks) {
  const Fixture f;
  const Factor factor(f.preintegrator, kWeight);
  const auto manifold = Factor::makeManifold();
  const Eigen::Matrix<double, Factor::kResidualDim, 1> zeros =
      Eigen::Matrix<double, Factor::kResidualDim, 1>::Zero();

  Eigen::Matrix<double, Factor::kResidualDim, 1> base;
  factor.residual(f.x.data(), f.input.data(), zeros.data(), base.data());

  // Tangent layout: [rho, omega | dv_j dbg_j dba_j | dv_i dbg_i dba_i].
  Eigen::Matrix<double, Factor::kTangentDim, 1> delta =
      Eigen::Matrix<double, Factor::kTangentDim, 1>::Zero();
  delta << 0.01, -0.02, 0.03, 0.004, 0.005, -0.006,         // pose_j
      0.01, 0.02, 0.03, 1e-4, 2e-4, 3e-4, 1e-3, 2e-3, 3e-3, // v/bg/ba_j
      -0.01, -0.02, -0.03, -1e-4, -2e-4, -3e-4, -1e-3, -2e-3, -3e-3; // _i

  Eigen::Matrix<double, Factor::kAmbientDim, 1> x_plus;
  manifold->plus(f.x.data(), delta.data(), x_plus.data());

  // Pose: left SE(3) increment. Everything else: additive, same order.
  const Eigen::Affine3d expected_pose =
      mslam::se3Exp(Eigen::Matrix<double, 6, 1>(delta.head<6>())) *
      mslam::poseFromStorage(f.x.data());
  EXPECT_NEAR(
      (mslam::poseFromStorage(x_plus.data()).matrix() - expected_pose.matrix())
          .norm(),
      0.0, 1e-12);
  EXPECT_NEAR((x_plus.tail<18>() - (f.x.tail<18>() + delta.tail<18>())).norm(),
              0.0, 1e-15);

  Eigen::Matrix<double, Factor::kResidualDim, 1> moved;
  factor.residual(x_plus.data(), f.input.data(), zeros.data(), moved.data());
  EXPECT_GT((moved - base).norm(), 1e-6);

  // Only state_i's extras: the prior block changes by exactly the weighted
  // step, and the preintegration block changes too (bias correction).
  Eigen::Matrix<double, Factor::kTangentDim, 1> delta_i =
      Eigen::Matrix<double, Factor::kTangentDim, 1>::Zero();
  delta_i.tail<9>() = delta.tail<9>();
  Eigen::Matrix<double, Factor::kAmbientDim, 1> x_i;
  manifold->plus(f.x.data(), delta_i.data(), x_i.data());
  EXPECT_EQ((x_i.head<21>() - f.x.head<21>()).norm(), 0.0);
  Eigen::Matrix<double, Factor::kResidualDim, 1> res_i;
  factor.residual(x_i.data(), f.input.data(), zeros.data(), res_i.data());
  const mslam::ImuPriorSigma sigma;
  Eigen::Matrix<double, 9, 1> sqrt_info;
  sqrt_info << Eigen::Vector3d::Constant(1.0 / sigma.velocity),
      Eigen::Vector3d::Constant(1.0 / sigma.gyro_bias),
      Eigen::Vector3d::Constant(1.0 / sigma.accel_bias);
  EXPECT_NEAR(((res_i.tail<9>() - base.tail<9>()) -
               kWeight * sqrt_info.cwiseProduct(delta.tail<9>()))
                  .norm(),
              0.0, 1e-9);
  EXPECT_GT((res_i.head<15>() - base.head<15>()).norm(), 1e-9);

  // Only state_j's velocity/biases: the prior block does not move.
  Eigen::Matrix<double, Factor::kTangentDim, 1> delta_j =
      Eigen::Matrix<double, Factor::kTangentDim, 1>::Zero();
  delta_j.segment<9>(6) = delta.segment<9>(6);
  Eigen::Matrix<double, Factor::kAmbientDim, 1> x_j;
  manifold->plus(f.x.data(), delta_j.data(), x_j.data());
  Eigen::Matrix<double, Factor::kResidualDim, 1> res_j;
  factor.residual(x_j.data(), f.input.data(), zeros.data(), res_j.data());
  EXPECT_EQ((res_j.tail<9>() - base.tail<9>()).norm(), 0.0);
  EXPECT_GT((res_j.head<15>() - base.head<15>()).norm(), 1e-9);
}

} // namespace
