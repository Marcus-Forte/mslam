#include "mslam/slam/SE3.hh"
#include "mslam/slam/SE3Manifold.hh"

#include <gtest/gtest.h>

#include <random>

namespace {

using Pose = Eigen::Matrix<double, 12, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

Eigen::Affine3d samplePose() {
  Vector6 twist;
  twist << 0.3, -1.2, 0.8, 0.5, -0.9, 1.4;
  return mslam::se3Exp(twist);
}

TEST(SE3Manifold, Sizes) {
  const mslam::SE3Manifold manifold;
  EXPECT_EQ(manifold.ambientDim(), 12U);
  EXPECT_EQ(manifold.tangentDim(), 6U);
}

TEST(SE3Manifold, StorageIsTheColumnMajor3x4Matrix) {
  const Eigen::Affine3d pose = samplePose();
  Pose x;
  mslam::poseToStorage(pose, x.data());
  const Eigen::Map<const Eigen::Matrix<double, 3, 4>> stored(x.data());
  EXPECT_EQ((stored - pose.matrix().topRows<3>()).norm(), 0.0);
  EXPECT_EQ((mslam::poseFromStorage(x.data()).matrix() - pose.matrix()).norm(),
            0.0);
}

TEST(SE3Manifold, PlusZeroReturnsThePose) {
  const mslam::SE3Manifold manifold;
  Pose x;
  mslam::poseToStorage(samplePose(), x.data());
  const Vector6 zero = Vector6::Zero();
  Pose x_plus;
  manifold.plus(x.data(), zero.data(), x_plus.data());
  EXPECT_EQ((x_plus - x).norm(), 0.0);
}

TEST(SE3Manifold, PlusIsTheLeftExponentialIncrement) {
  const mslam::SE3Manifold manifold;
  const Eigen::Affine3d pose = samplePose();
  Pose x;
  mslam::poseToStorage(pose, x.data());

  std::mt19937 rng(1);
  std::normal_distribution<double> gaussian(0.0, 0.7);
  for (int i = 0; i < 20; ++i) {
    Vector6 delta;
    for (int k = 0; k < 6; ++k) {
      delta[k] = gaussian(rng);
    }
    Pose x_plus;
    manifold.plus(x.data(), delta.data(), x_plus.data());
    const Eigen::Affine3d expected = mslam::se3Exp(delta) * pose;
    EXPECT_NEAR(
        (mslam::poseFromStorage(x_plus.data()).matrix() - expected.matrix())
            .norm(),
        0.0, 1e-12);
  }
}

TEST(SE3Manifold, RotationStaysOrthonormalAfterManySteps) {
  const mslam::SE3Manifold manifold;
  Pose x;
  mslam::poseToStorage(samplePose(), x.data());

  std::mt19937 rng(2);
  std::normal_distribution<double> gaussian(0.0, 0.3);
  for (int i = 0; i < 100000; ++i) {
    Vector6 delta;
    for (int k = 0; k < 6; ++k) {
      delta[k] = gaussian(rng);
    }
    Pose x_plus;
    manifold.plus(x.data(), delta.data(), x_plus.data());
    x = x_plus;
  }
  const Eigen::Matrix3d R =
      Eigen::Map<const Eigen::Matrix<double, 3, 4>>(x.data()).leftCols<3>();
  EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0,
              1e-10);
  EXPECT_NEAR(R.determinant(), 1.0, 1e-10);
}

} // namespace
