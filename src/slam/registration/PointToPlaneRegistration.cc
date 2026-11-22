#include "mslam/slam/registration/PointToPlaneRegistration.hh"
#include "moptim/AnalyticalCost.hh"
#include "moptim/LevenbergMarquardt.hh"
#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/PointDistance.hh"
#include <Eigen/Dense>

namespace mslam {

PointToPlaneRegistration::PointToPlaneRegistration(
    int num_registration_iterations, int num_optimizer_iterations,
    float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
    std::shared_ptr<const ParallelExecutor> executor)
    : icp_(IcpParameters{num_registration_iterations, num_optimizer_iterations,
                         max_correspondence_distance},
           std::move(logger), std::move(correspondence_finder),
           std::move(executor)) {}

SlamState PointToPlaneRegistration::Align(const SlamState &state,
                                          const IMap &map,
                                          const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;
  const auto &parameters = icp_.parameters();

  first_round_ = {};
  auto total_T = toAffine(state);
  icp_.begin(map, scan, total_T);

  // The optimizer variable is the pose itself, stepped by the manifold.
  Eigen::Matrix<double, SE3Manifold::kAmbientDim, 1> pose;
  poseToStorage(total_T, pose.data());

  moptim::LevenbergMarquardt<double> lm(manifold_);
  lm.setMaxIterations(parameters.optimizer_iterations);
  icp_.attachObserver(lm);

  int small_delta_hits = 0;
  for (int i = 0; i < parameters.registration_iterations; ++i) {
    if (icp_.findPlaneConstraints(map) == 0) {
      icp_.logger().warn(
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    const auto solve_start = std::chrono::steady_clock::now();
    lm.clearCosts();
    // The source cloud is the scan moved by the pose of the start of the round.
    lm.addCost(
        std::make_shared<moptim::AnalyticalCost<Point3PlaneDistance, double>>(
            icp_.planeInputs().front().data(), icp_.planeOffsets().data(),
            icp_.planeOffsets().size(), 6, 1, manifold_,
            Point3PlaneDistance(total_T), /*active_param_dim=*/6));
    const Eigen::Affine3d pose_before = total_T;
    lm.optimize(pose.data());
    icp_.timings().solve += std::chrono::steady_clock::now() - solve_start;

    total_T = poseFromStorage(pose.data());
    if (i == 0) {
      const Eigen::Matrix<double, 6, 1> step =
          se3Log(Eigen::Affine3d(total_T * pose_before.inverse()));
      first_round_ = {true, step.head<3>().norm(), step.tail<3>().norm()};
    }
    icp_.resetSource(total_T);

    icp_.logger().debug("Registration Iteration: {}/{}", i + 1,
                        parameters.registration_iterations);

    if (isNegligibleRoundStep(pose_before, total_T)) {
      if (++small_delta_hits > k_maxSmallDeltaHits) {
        break;
      }
    }
  }

  return toSlamState(total_T);
}

} // namespace mslam
