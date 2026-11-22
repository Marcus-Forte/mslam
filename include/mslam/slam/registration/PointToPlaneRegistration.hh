#pragma once

#include "mslam/slam/SE3Manifold.hh"
#include "mslam/slam/registration/IRegistration.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"

namespace mslam {

/// Point-to-plane ICP: minimizes the distance of each scan point to the plane
/// through its map correspondence (normal from a KNN neighbourhood), with an
/// analytical jacobian. The pose itself is the optimizer variable
/// (SE3Manifold).
class PointToPlaneRegistration : public IRegistration {
public:
  /// @param executor Runs the normal estimation (nullptr: sequential); give
  /// @p correspondence_finder the same one to parallelize the search too.
  /// @param backend Where the solves run (IcpWorkspace::useDeviceOptimizer()).
  PointToPlaneRegistration(
      int num_registration_iterations, int num_optimizer_iterations,
      float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
      std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
      std::shared_ptr<const ParallelExecutor> executor = nullptr,
      ComputeBackend backend = ComputeBackend::Host);

  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;

  /// The pose change the first round of the last Align() applied (invalid if
  /// that round found no correspondences).
  const FirstRoundIncrement &lastFirstRound() const { return first_round_; }

  /// Time spent in the parts of every Align so far.
  const RegistrationTimings &timings() const { return icp_.timings(); }

  PointToPlaneRegistration(const PointToPlaneRegistration &) = delete;
  PointToPlaneRegistration &operator=(const PointToPlaneRegistration &) =
      delete;
  ~PointToPlaneRegistration() override;

private:
  /// The optimizer and its cost, kept from one round and Align to the next
  /// (PointToPlaneRegistration.cc).
  struct Solver;

  IcpWorkspace icp_;
  std::unique_ptr<Solver> solver_;
  /// Shared with the optimizer and its costs (they must use the same object).
  std::shared_ptr<const SE3Manifold> manifold_ =
      std::make_shared<SE3Manifold>();
  FirstRoundIncrement first_round_;
};

} // namespace mslam
