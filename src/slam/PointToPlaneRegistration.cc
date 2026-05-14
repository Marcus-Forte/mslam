#include "slam/registration/PointToPlaneRegistration.hh"
#include "Timer.hh"
#include "moptim/AnalyticalCost.h"
#include "moptim/LevenbergMarquardt.h"
#include "moptim/NumericalCostForwardEuler.h"
#include "slam/NormalEstimator.hh"
#include "slam/Transform.hh"
#include "slam/registration/PointDistance.hh"
#include <Eigen/Dense>

namespace mslam {

Pose3D PointToPlaneRegistration::Align(const Pose3D &pose, const IMap &map,
                                       const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;
  NormalEstimator normal_estimator(map);
  Eigen::VectorXd x{{pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]}};

  std::vector<Eigen::Matrix<double, 6, 1>> inputs;
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

    Timer normal_timer;
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
    moptim::LevenbergMarquardt<double> lm(6, logger_);
    lm.setMaxIterations(num_optimizer_iterations_);
    lm.addCost(
        std::make_shared<moptim::AnalyticalCost<Point3PlaneDistance, double>>(
            inputs[0].data(), map_points[0].data(), map_points.size(), 6, 3,
            6));
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
