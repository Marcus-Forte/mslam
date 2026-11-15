#include "mslam/slam/registration/PointToPointRegistration.hh"
#include "moptim/LevenbergMarquardt.hh"
#include "moptim/NumericalCostForwardEuler.hh"
#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/PointDistance.hh"
#include <Eigen/Dense>

namespace mslam {

PointToPointRegistration::PointToPointRegistration(
    int num_registration_iterations, int num_optimizer_iterations,
    float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder)
    : icp_(IcpParameters{num_registration_iterations, num_optimizer_iterations,
                         max_correspondence_distance},
           std::move(logger), std::move(correspondence_finder)) {}

SlamState PointToPointRegistration::Align(const SlamState &state,
                                          const IMap &map,
                                          const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;
  const auto &parameters = icp_.parameters();

  int small_delta_hits = 0;

  auto total_T = toAffine(state);
  icp_.begin(map, scan, total_T);

  Eigen::Matrix<double, SE3Manifold::kAmbientDim, 1> pose;
  poseToStorage(total_T, pose.data());

  moptim::LevenbergMarquardt<double> lm(manifold_);
  lm.setMaxIterations(parameters.optimizer_iterations);
  icp_.attachObserver(lm);

  for (int i = 0; i < parameters.registration_iterations; ++i) {
    icp_.findCorrespondences(map);

    inputs_buffer_.clear();
    map_points_buffer_.clear();
    for (const auto &[scan_point, map_point] : icp_.correspondences()) {
      inputs_buffer_.emplace_back(scan_point.x, scan_point.y, scan_point.z);
      map_points_buffer_.emplace_back(map_point.x, map_point.y, map_point.z);
    }

    if (map_points_buffer_.empty()) {
      icp_.logger().warn(
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    lm.clearCosts();
    // The source cloud is the scan moved by the pose of the start of the round.
    lm.addCost(std::make_shared<
               moptim::NumericalCostForwardEuler<Point3Distance, double>>(
        inputs_buffer_[0].data(), map_points_buffer_[0].data(),
        map_points_buffer_.size(), 3, 3, manifold_, Point3Distance(total_T)));
    const auto result = lm.optimize(pose.data());

    total_T = poseFromStorage(pose.data());
    icp_.resetSource(total_T);

    icp_.logger().debug("Registration Iteration: {}/{}", i + 1,
                        parameters.registration_iterations);

    if (result.status == moptim::Status::SMALL_DELTA) {
      if (++small_delta_hits > k_maxSmallDeltaHits) {
        break;
      }
    }
  }

  return toSlamState(total_T);
}

} // namespace mslam
