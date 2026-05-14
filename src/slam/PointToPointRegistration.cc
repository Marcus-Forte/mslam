#include "slam/registration/PointToPointRegistration.hh"
#include "Timer.hh"
#include "moptim/LevenbergMarquardt.h"
#include "moptim/NumericalCostForwardEuler.h"
#include "slam/Transform.hh"
#include "slam/registration/PointDistance.hh"
#include <Eigen/Dense>

namespace mslam {

Pose3D PointToPointRegistration::Align(const Pose3D &pose, const IMap &map,
                                       const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;
  Eigen::VectorXd x{{pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]}};

  std::vector<Eigen::Vector3d> inputs;
  VectorPoint3d map_points;
  int small_delta_hits = 0;
  Timer iteration_timer;
  Timer stage_timer;
  PointCloud transformed_scan;

  inputs.reserve(scan.size());
  map_points.reserve(scan.size());

  for (int i = 0; i < num_registration_iterations_; ++i) {
    iteration_timer.start();
    const auto transform = toAffine(x[0], x[1], x[2], x[3], x[4], x[5]);

    transformed_scan = scan;
    transformCloud(transform, transformed_scan);

    const auto correspondences = correspondence_finder_->find(
        map, scan, transformed_scan, max_correspondence_distance_);

    inputs.clear();
    map_points.clear();
    for (const auto &[scan_point, map_point] : correspondences) {
      inputs.emplace_back(scan_point.x, scan_point.y, scan_point.z);
      map_points.emplace_back(map_point.x, map_point.y, map_point.z);
    }

    if (map_points.empty()) {
      logger_->log(
          ILog::Level::WARNING,
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    stage_timer.start();
    moptim::LevenbergMarquardt<double> lm(6, logger_);
    lm.setMaxIterations(num_optimizer_iterations_);
    lm.addCost(std::make_shared<
               moptim::NumericalCostForwardEuler<Point3Distance, double>>(
        inputs[0].data(), map_points[0].data(), map_points.size(), 3, 3, 6));
    const auto status = lm.optimize(x.data());

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

  return {x[0], x[1], x[2], x[3], x[4], x[5]};
}

} // namespace mslam
