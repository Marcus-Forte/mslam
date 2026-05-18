#include "slam/registration/PointToPlaneRegistration.hh"
#include "Timer.hh"
#include "moptim/LevenbergMarquardt.h"
#include "moptim/NumericalCostForwardEuler.h"
#include "moptim/PlusOperations/SE3PlusOperator.h"
#include "slam/NormalEstimator.hh"
#include "slam/Transform.hh"
#include "slam/registration/PointDistance.hh"
#include <Eigen/Dense>

namespace mslam {

SlamState PointToPlaneRegistration::Align(const SlamState &state,
                                          const IMap &map,
                                          const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;
  NormalEstimator normal_estimator(map);
  auto total_T =
      toAffine(state.position.x(), state.position.y(), state.position.z(),
               state.rotation.x(), state.rotation.y(), state.rotation.z());

  std::vector<Eigen::Matrix<double, 6, 1>> inputs;
  VectorPoint3d map_points;
  int small_delta_hits = 0;
  Timer iteration_timer;
  Timer stage_timer;
  Timer normal_timer;

  inputs.reserve(scan.size());
  map_points.reserve(scan.size());

  // Initial guess
  auto source = scan;
  transformCloud(total_T, source);

  Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();

  moptim::LevenbergMarquardt<double, moptim::SE3PlusOperator<double>> lm(
      6, logger_);
  lm.setMaxIterations(num_optimizer_iterations_);

  for (int i = 0; i < num_registration_iterations_; ++i) {
    iteration_timer.start();
    const auto correspondences =
        correspondence_finder_->find(map, source, max_correspondence_distance_);

    normal_timer.start();
    inputs.clear();
    map_points.clear();
    for (const auto &[scan_point, map_point] : correspondences) {
      const auto normal = normal_estimator.estimate(map_point);
      if (!normal.has_value()) {
        continue;
      }

      inputs.emplace_back(scan_point.x, scan_point.y, scan_point.z, normal->x(),
                          normal->y(), normal->z());
      map_points.emplace_back(map_point.x, map_point.y, map_point.z);
    }
    logger_->log(
        ILog::Level::DEBUG, "Normal estimation. {} / {} points. Took: {} us",
        map_points.size(), correspondences.size(), normal_timer.stop());

    if (map_points.empty()) {
      logger_->log(
          ILog::Level::WARNING,
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    stage_timer.start();
    delta.setZero();
    lm.clearCosts();
    lm.addCost(
        std::make_shared<moptim::NumericalCostForwardEuler<
            Point3PlaneDistance, double, moptim::SE3PlusOperator<double>>>(
            inputs[0].data(), map_points[0].data(), map_points.size(), 6, 3,
            6));
    const auto status = lm.optimize(delta.data());

    const auto delta_T = moptim::se3Exp(delta);
    transformCloud(delta_T, source);
    total_T = delta_T * total_T;

    logger_->log(ILog::Level::DEBUG, "Opt. Took: {} us", stage_timer.stop());
    logger_->log(ILog::Level::DEBUG,
                 "Registration Iteration: {}/{}. Total: {} us", i + 1,
                 num_registration_iterations_, iteration_timer.stop());

    if (status == moptim::Status::SMALL_DELTA) {
      if (++small_delta_hits > k_maxSmallDeltaHits) {
        break;
      }
    }
  }

  const auto t = total_T.translation();
  const auto &R = total_T.linear();
  return SlamState{
      .position = t,
      .rotation = {std::atan2(-R(1, 2), R(2, 2)),
                   std::asin(std::clamp(R(0, 2), -1.0, 1.0)),
                   std::atan2(-R(0, 1), R(0, 0))},
  };
}

} // namespace mslam
