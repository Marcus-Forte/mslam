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
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder)
    : icp_(IcpParameters{num_registration_iterations, num_optimizer_iterations,
                         max_correspondence_distance},
           std::move(logger), std::move(correspondence_finder)) {}

SlamState PointToPlaneRegistration::Align(const SlamState &state,
                                          const IMap &map,
                                          const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;
  const auto &parameters = icp_.parameters();

  auto total_T = toAffine(state);
  icp_.begin(map, scan, total_T);

  Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();

  moptim::LevenbergMarquardt<double> lm(6);
  lm.setMaxIterations(parameters.optimizer_iterations);
  icp_.attachObserver(lm);

  int small_delta_hits = 0;
  for (int i = 0; i < parameters.registration_iterations; ++i) {
    if (icp_.findPlaneConstraints(map) == 0) {
      icp_.logger().warn(
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    delta.setZero();
    lm.clearCosts();
    lm.addCost(
        std::make_shared<moptim::AnalyticalCost<Point3PlaneDistance, double>>(
            icp_.planeInputs().front().data(), icp_.planeOffsets().data(),
            icp_.planeOffsets().size(), 6, 1, 6));
    const auto result = lm.optimize(delta.data());

    const auto delta_T = se3Exp(delta);
    icp_.moveSource(delta_T);
    total_T = delta_T * total_T;

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
