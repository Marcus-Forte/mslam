#include "mslam/slam/Transform.hh"

#include <gtest/gtest.h>

#include <random>

namespace {

TEST(Transform, EulerRoundTripsThroughRotation) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> angle(-3.0, 3.0);
  std::uniform_real_distribution<double> pitch(-1.5, 1.5); // |ry| < pi/2
  for (int i = 0; i < 200; ++i) {
    const Eigen::Vector3d euler(angle(rng), pitch(rng), angle(rng));
    const Eigen::Vector3d recovered =
        mslam::rotationToEuler(mslam::eulerToRotation(euler));
    // Compare as rotations: angles may differ by multiples of 2 pi.
    EXPECT_NEAR(
        (mslam::eulerToRotation(recovered) - mslam::eulerToRotation(euler))
            .norm(),
        0.0, 1e-12);
    EXPECT_NEAR(recovered.y(), euler.y(), 1e-9);
  }
}

TEST(Transform, RotationToEulerIsConsistentWithToAffine) {
  const Eigen::Vector3d euler(0.3, -0.4, 1.2);
  const auto pose =
      mslam::toAffine(1.0, 2.0, 3.0, euler.x(), euler.y(), euler.z());
  EXPECT_NEAR((mslam::rotationToEuler(pose.linear()) - euler).norm(), 0.0,
              1e-12);
  EXPECT_NEAR((mslam::eulerToRotation(euler) - pose.linear()).norm(), 0.0,
              1e-15);
}

TEST(Transform, StateConversionsAreInverse) {
  mslam::SlamState state;
  state.position = {1.0, -2.0, 0.5};
  state.rotation = {0.1, -0.2, 0.3};
  state.velocity = {9.0, 9.0, 9.0};

  const auto pose = mslam::toAffine(state);
  EXPECT_NEAR((pose.translation() - state.position).norm(), 0.0, 1e-15);

  const auto rebuilt = mslam::toSlamState(pose);
  EXPECT_NEAR((rebuilt.position - state.position).norm(), 0.0, 1e-12);
  EXPECT_NEAR((rebuilt.rotation - state.rotation).norm(), 0.0, 1e-12);
  // Only the pose is carried.
  EXPECT_EQ(rebuilt.velocity, Eigen::Vector3d::Zero());

  mslam::SlamState target = state;
  mslam::setPose(target, mslam::toAffine(0.0, 0.0, 0.0, 0.0, 0.0, 0.5));
  EXPECT_NEAR(target.rotation.z(), 0.5, 1e-12);
  EXPECT_EQ(target.position, Eigen::Vector3d::Zero());
  // Velocity and biases are untouched.
  EXPECT_EQ(target.velocity, state.velocity);
}

TEST(Transform, TransformCloudAppliesThePose) {
  mslam::PointCloud cloud;
  cloud.emplace_back(1.0F, 0.0F, 0.0F, 7.0F);
  mslam::transformCloud(mslam::toAffine(1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 2),
                        cloud);
  EXPECT_NEAR(cloud[0].x, 1.0F, 1e-6F);
  EXPECT_NEAR(cloud[0].y, 3.0F, 1e-6F);
  EXPECT_NEAR(cloud[0].z, 3.0F, 1e-6F);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 7.0F);
}

} // namespace
