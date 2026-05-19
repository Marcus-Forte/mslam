#pragma once

#include "IRegistration.hh"
#include "slam/ImuPreintegration.hh"

#include <Eigen/Dense>

namespace mslam {

/// Registration combining point-to-plane scan matching with an IMU
/// preintegration factor in a joint 15-DOF optimization.
class ImuRegistration : public IRegistration {
public:
  using IRegistration::IRegistration;

  /// Standard Align (no IMU — not supported, throws).
  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;

  /// Joint 15-DOF Align: optimizes pose, velocity, and biases simultaneously.
  /// @param state        Current state estimate (initial guess)
  /// @param map          Reference map for correspondences
  /// @param scan         Current scan in body frame
  /// @param prev_state   Previous optimized state (constant in optimization)
  /// @param preintegrator Accumulated IMU measurements since prev_state
  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan, const SlamState &prev_state,
                  const ImuPreintegrator &preintegrator);

private:
  std::vector<Eigen::Matrix<double, 6, 1>> inputs_buffer_;
  VectorPoint3d map_points_buffer_;
};

} // namespace mslam
