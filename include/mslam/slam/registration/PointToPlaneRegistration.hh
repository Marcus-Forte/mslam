#pragma once

#include "mslam/slam/registration/IRegistration.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"

namespace mslam {

/// Point-to-plane ICP: minimizes the distance of each scan point to the plane
/// through its map correspondence (normal from a KNN neighbourhood), with an
/// analytical se(3) jacobian.
class PointToPlaneRegistration : public IRegistration {
public:
  PointToPlaneRegistration(
      int num_registration_iterations, int num_optimizer_iterations,
      float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
      std::shared_ptr<ICorrespondenceFinder> correspondence_finder);

  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;

private:
  IcpWorkspace icp_;
};

} // namespace mslam
