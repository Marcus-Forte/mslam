#include "slam/Registration.hh"
#include "LevenbergMarquardt.hh"
#include "NumericalCost.hh"
#include "Timer.hh"
#include "common/Points.hh"
#include "slam/Transform.hh"
#include <memory>

#include "PointDistance.hh"

namespace mslam {

Registration::Registration(int num_registration_iterations,
                           int num_optimizer_iterations,
                           float max_correspondence_distance,
                           const std::shared_ptr<ILog> &logger)
    : num_registration_iterations_(num_registration_iterations),
      num_optimizer_iterations_(num_optimizer_iterations),
      max_correspondence_distance_(max_correspondence_distance),
      logger_(logger) {}

/// \todo add transformed scan as output
Pose2D Registration::Align2D(const Pose2D &pose, const IMap &map,
                             const PointCloud3 &scan) {

  VectorPoint2d scan_points;
  VectorPoint2d map_correspondences;

  // Initial guess
  Eigen::VectorXd x{{pose[0], pose[1], pose[2]}};
  Eigen::Affine2d transform_;

  static Timer timer;

  for (int i = 0; i < num_registration_iterations_; ++i) {
    timer.start();
    transform_ = toAffine(x[0], x[1], x[2]);
    // Data association

    scan_points.clear();
    map_correspondences.clear();
    scan_points.reserve(scan.size());
    map_correspondences.reserve(scan.size());

    // Find point correspondences
    for (const auto &pt : scan) {
      const Eigen::Vector2d point_eigen{pt.x, pt.y};
      const Eigen::Vector2d transformed_scan_pt = transform_ * point_eigen;

      /// \todo optimize
      const Point3 query(transformed_scan_pt.x(), transformed_scan_pt.y(), 0);
      const auto closest = map.getClosestNeighbor(query);
      if (closest.second <
          max_correspondence_distance_ * max_correspondence_distance_) {

        scan_points.emplace_back(pt.x, pt.y); // untransformed observation
        map_correspondences.emplace_back(closest.first.x,
                                         closest.first.y); // point in map
      }
    }

    // Align
    LevenbergMarquardt lm(logger_);

    lm.setMaxIterations(num_optimizer_iterations_);

    auto model = std::make_shared<Point2Distance>();

    auto cost = std::make_shared<NumericalCost>(
        map_correspondences[0].data(), scan_points[0].data(),
        map_correspondences.size(), 2, 3, model);

    lm.addCost(cost);
    const auto status = lm.optimize(x);

    const auto delta = timer.stop();
    logger_->log(
        ILog::Level::INFO,
        "Reg. Iteration: {}/ {}. Correspondences: {} / {}. Took: {} us", i + 1,
        num_optimizer_iterations_, map_correspondences.size(), scan.size(),
        delta);

    if (registration_callback2d_) {
      registration_callback2d_({x[0], x[1], x[2]}, map_correspondences,
                               scan_points);
    }
    /// \todo status convergence heuristic
    if (status == IOptimizer::Status::SMALL_DELTA) {
      break;
    }
  }

  return {x[0], x[1], x[2]};
}

Pose3D Registration::Align3D(const Pose3D &pose, const IMap &map,
                             const PointCloud3 &scan) {

  throw std::runtime_error("Align3D not implemented");
}

void Registration::registerIterationCallback2D(
    RegistrationCallback2D &&callback) {
  registration_callback2d_ = callback;
}
} // namespace mslam