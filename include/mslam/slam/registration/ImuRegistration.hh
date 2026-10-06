#pragma once

#include "mslam/slam/ImuPreintegration.hh"
#include "mslam/slam/registration/IRegistration.hh"

#include <Eigen/Dense>

namespace mslam {

/// Registration combining point-to-plane scan matching with an IMU
/// preintegration factor in a joint optimization of the new state (pose,
/// velocity, biases) and the previous state's velocity and biases.
class ImuRegistration : public IRegistration {
public:
  using IRegistration::IRegistration;

  /// Standard Align (no IMU - not supported, throws).
  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;

  /// Joint Align: optimizes pose, velocity, and biases of the new state, plus
  /// velocity and biases of the previous state (its pose stays fixed).
  /// @param state        Current state estimate (initial guess)
  /// @param map          Reference map for correspondences
  /// @param scan         Current scan in body frame
  /// @param prev_state   Previous optimized state (pose constant; velocity and
  ///                     biases are the prior mean for their re-estimation)
  /// @param preintegrator Accumulated IMU measurements since prev_state
  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan, const SlamState &prev_state,
                  const ImuPreintegrator &preintegrator);

private:
  std::vector<Eigen::Matrix<double, 6, 1>> inputs_buffer_;
  VectorPoint3d map_points_buffer_;
};

} // namespace mslam
