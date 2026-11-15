#pragma once

#include "mslam/slam/SE3Manifold.hh"
#include "mslam/slam/registration/IRegistration.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"

#include <Eigen/Dense>
#include <vector>

namespace mslam {

class PointToPointRegistration : public IRegistration {
public:
  PointToPointRegistration(
      int num_registration_iterations, int num_optimizer_iterations,
      float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
      std::shared_ptr<ICorrespondenceFinder> correspondence_finder);

  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;

private:
  IcpWorkspace icp_;
  /// Shared with the optimizer and its costs (they must use the same object).
  std::shared_ptr<const SE3Manifold> manifold_ =
      std::make_shared<SE3Manifold>();
  std::vector<Eigen::Vector3d> inputs_buffer_;
  VectorPoint3d map_points_buffer_;
};

} // namespace mslam
