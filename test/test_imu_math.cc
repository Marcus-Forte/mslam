#include "mslam/slam/ImuMath.hh"
#include <cmath>
#include <gtest/gtest.h>

namespace mslam {
namespace {

msensor::IMUData makeAccel(double ax, double ay, double az) {
  msensor::IMUData data{};
  data.ax = ax;
  data.ay = ay;
  data.az = az;
  return data;
}

TEST(ImuMathTest, LevelGravityGivesZeroRollPitch) {
  const auto roll_pitch =
      estimateGravityAlignedRollPitch(makeAccel(0.0, 0.0, k_gravity_mps2));
  ASSERT_TRUE(roll_pitch.has_value());
  EXPECT_NEAR(roll_pitch->x(), 0.0, 1e-5);
  EXPECT_NEAR(roll_pitch->y(), 0.0, 1e-5);
}

TEST(ImuMathTest, TiltedGravityGivesRollAndPitch) {
  const double angle = 0.3;
  const auto roll = estimateGravityAlignedRollPitch(
      makeAccel(0.0, std::sin(angle), std::cos(angle)));
  ASSERT_TRUE(roll.has_value());
  EXPECT_NEAR(roll->x(), angle, 1e-5);
  EXPECT_NEAR(roll->y(), 0.0, 1e-5);

  const auto pitch = estimateGravityAlignedRollPitch(
      makeAccel(-std::sin(angle), 0.0, std::cos(angle)));
  ASSERT_TRUE(pitch.has_value());
  EXPECT_NEAR(pitch->x(), 0.0, 1e-5);
  EXPECT_NEAR(pitch->y(), angle, 1e-5);
}

TEST(ImuMathTest, RejectsDegenerateAcceleration) {
  EXPECT_FALSE(
      estimateGravityAlignedRollPitch(makeAccel(0.0, 0.0, 0.0)).has_value());
  EXPECT_FALSE(
      estimateGravityAlignedRollPitch(makeAccel(NAN, 0.0, k_gravity_mps2))
          .has_value());
}

TEST(ImuMathTest, GravityIsRemovedWhenLevel) {
  const auto world = toGravityCompensatedWorldAcceleration(
      Eigen::Vector3d::Zero(), makeAccel(0.0, 0.0, k_gravity_mps2), 1.0);
  EXPECT_NEAR(world.norm(), 0.0, 1e-5);
}

TEST(ImuMathTest, AccelerationScaleIsApplied) {
  // A unit-g sensor scaled to m/s^2 should also cancel gravity.
  const auto world = toGravityCompensatedWorldAcceleration(
      Eigen::Vector3d::Zero(), makeAccel(0.0, 0.0, 1.0), k_gravity_mps2);
  EXPECT_NEAR(world.norm(), 0.0, 1e-5);
}

TEST(ImuMathTest, AccelerationIsRotatedIntoWorldFrame) {
  // Yaw by 90 degrees maps body +x to world +y.
  const auto world = toGravityCompensatedWorldAcceleration(
      Eigen::Vector3d(0.0, 0.0, M_PI / 2.0),
      makeAccel(1.0, 0.0, k_gravity_mps2), 1.0);
  EXPECT_NEAR(world.x(), 0.0, 1e-5);
  EXPECT_NEAR(world.y(), 1.0, 1e-5);
  EXPECT_NEAR(world.z(), 0.0, 1e-5);
}

} // namespace
} // namespace mslam
