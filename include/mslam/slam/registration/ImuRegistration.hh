#pragma once

#include "mslam/common/State.hh"
#include "mslam/slam/ImuPreintegration.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"

#include "moptim/Manifold.hh"

#include <Eigen/Dense>
#include <memory>

namespace mslam {

/// Registration combining point-to-plane scan matching with an IMU
/// preintegration factor in a joint optimization of the new state (pose,
/// velocity, biases) and the previous state's velocity and biases.
///
/// It is deliberately not an IRegistration: it needs the previous state and
/// the preintegrated IMU measurements, so it has no plain
/// (state, map, scan) alignment.
class ImuRegistration {
public:
  using FirstRoundIncrement = mslam::FirstRoundIncrement;

  /// @param sycl_threads Work-groups of the device search kernel (0: every
  /// compute unit).
  ImuRegistration(int num_registration_iterations, int num_optimizer_iterations,
                  float max_correspondence_distance,
                  std::shared_ptr<spdlog::logger> logger,
                  std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
                  ComputeBackend backend = ComputeBackend::Host,
                  unsigned sycl_threads = 0);

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

  const FirstRoundIncrement &lastFirstRound() const { return first_round_; }

  /// Time spent in the parts of every Align so far.
  const RegistrationTimings &timings() const { return icp_.timings(); }

  ImuRegistration(const ImuRegistration &) = delete;
  ImuRegistration &operator=(const ImuRegistration &) = delete;
  ~ImuRegistration();

private:
  /// The optimizer and its costs, kept from one round and scan to the next
  /// (ImuRegistration.cc).
  struct Solver;

  IcpWorkspace icp_;
  std::unique_ptr<Solver> solver_;
  /// SE3Manifold x Euclidean(18), shared with the optimizer and its costs.
  std::shared_ptr<const moptim::IManifold<double>> manifold_;
  FirstRoundIncrement first_round_;
};

} // namespace mslam
