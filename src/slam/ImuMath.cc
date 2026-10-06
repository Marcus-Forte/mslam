#include "mslam/slam/ImuMath.hh"
#include "mslam/slam/Transform.hh"
#include <cmath>

namespace mslam {

namespace {
constexpr double k_min_acceleration_norm_mps2 = 1e-3;
} // namespace

Eigen::Vector3d
toGravityCompensatedWorldAcceleration(const Eigen::Vector3d &rotation,
                                      const msensor::IMUData &imu_data,
                                      double acceleration_scale) {
  const auto orientation =
      toAffine(0.0, 0.0, 0.0, rotation.x(), rotation.y(), rotation.z())
          .linear();
  Eigen::Vector3d world_acceleration =
      orientation * (acceleration_scale *
                     Eigen::Vector3d(imu_data.ax, imu_data.ay, imu_data.az));
  world_acceleration.z() -= k_gravity_mps2;
  return world_acceleration;
}

std::optional<Eigen::Vector2d>
estimateGravityAlignedRollPitch(const msensor::IMUData &imu_data) {
  const Eigen::Vector3d body_acceleration(imu_data.ax, imu_data.ay,
                                          imu_data.az);
  const double acceleration_norm = body_acceleration.norm();
  if (!std::isfinite(acceleration_norm) ||
      acceleration_norm < k_min_acceleration_norm_mps2) {
    return std::nullopt;
  }

  const Eigen::Vector3d gravity_direction =
      body_acceleration / acceleration_norm;
  const double roll =
      std::atan2(gravity_direction.y(),
                 std::hypot(gravity_direction.x(), gravity_direction.z()));
  const double pitch =
      std::atan2(-gravity_direction.x(), gravity_direction.z());
  return Eigen::Vector2d(roll, pitch);
}

} // namespace mslam
