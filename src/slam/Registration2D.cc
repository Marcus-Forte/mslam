#include "slam/Registration2D.hh"
#include "LevenbergMarquardt.hh"
#include "NumericalCost.hh"
#include "common/Points.hh"
#include "slam/Transform.hh"
#include <memory>

namespace mslam {

struct Model {
  Model(const Eigen::VectorXd &x) : transform_{toAffine(x[0], x[1], x[2])} {}

  // Error function. Input -> Correspondent point from current Map. Observation
  // := Point from new scan.
  Eigen::Vector2d operator()(const Eigen::Vector2d &input,
                             const Eigen::Vector2d &observation) {
    return input - transform_ * observation;
  }

private:
  Eigen::Affine2d transform_;
};

Registration2D::Registration2D(int num_registration_iterations,
                               int num_optimizer_iterations,
                               float max_correspondence_distance,
                               const std::shared_ptr<ILog> &logger)
    : num_registration_iterations_(num_registration_iterations),
      num_optimizer_iterations_(num_optimizer_iterations),
      max_correspondence_distance_(max_correspondence_distance),
      logger_(logger) {}

Pose2D Registration2D::Align(const Pose2D &pose, const IMap &map,
                             const PointCloud3 &scan) {

  VectorPoint2d scan_points;
  VectorPoint2d map_correspondences;

  // Initial guess
  Eigen::VectorXd x{{pose[0], pose[1], pose[2]}};
  Eigen::Affine2d transform_;

  for (int i = 0; i < num_registration_iterations_; ++i) {
    transform_ = toAffine(x[0], x[1], x[2]);
    // Data association

    scan_points.clear();
    map_correspondences.clear();
    scan_points.reserve(scan.size());
    map_correspondences.reserve(scan.size());

    // Find point correspondences
    for (const auto &pt : scan) {
      const Eigen::Vector2d point_eigen{pt.x, pt.y};
      const auto transformed_scan_pt = transform_ * point_eigen;
      /// \todo optimize
      const Point3 query(transformed_scan_pt.x(), transformed_scan_pt.y(), 0);
      const auto closest = map.getClosestNeighbor(query);
      if (closest.second <
          max_correspondence_distance_ * max_correspondence_distance_) {

        scan_points.emplace_back(pt.x, pt.y); // observation
        map_correspondences.emplace_back(closest.first.x,
                                         closest.first.y); // point in map
      }
    }
    // Align
    logger_->log(ILog::Level::DEBUG, "Correspondences: {}",
                 map_correspondences.size());
    LevenbergMarquardt lm(logger_);
    lm.setMaxIterations(num_optimizer_iterations_);
    auto cost = std::make_shared<
        NumericalCost<Eigen::Vector2d, Eigen::Vector2d, Model>>(
        &map_correspondences, &scan_points);

    lm.addCost(cost);
    lm.optimize(x);
  }

  return {x[0], x[1], x[2]};
}

} // namespace mslam