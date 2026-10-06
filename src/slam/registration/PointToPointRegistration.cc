#include "mslam/slam/registration/PointToPointRegistration.hh"
#include "moptim/LevenbergMarquardt.hh"
#include "moptim/NumericalCostForwardEuler.hh"
#include "mslam/OptimizerObserver.hh"
#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/PointDistance.hh"
#include <Eigen/Dense>

namespace mslam {

SlamState PointToPointRegistration::Align(const SlamState &state,
                                          const IMap &map,
                                          const PointCloud &scan) {
  static constexpr int k_maxSmallDeltaHits = 3;

  int small_delta_hits = 0;

  auto total_T =
      toAffine(state.position.x(), state.position.y(), state.position.z(),
               state.rotation.x(), state.rotation.y(), state.rotation.z());
  source_buffer_ = scan;
  transformCloud(total_T, source_buffer_);

  inputs_buffer_.reserve(scan.size());
  map_points_buffer_.reserve(scan.size());

  moptim::LevenbergMarquardt<double> lm(6);
  lm.setMaxIterations(num_optimizer_iterations_);
  OptimizerObserver<double> observer(logger_);
  lm.setObserver(&observer);

  Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();

  for (int i = 0; i < num_registration_iterations_; ++i) {
    correspondence_finder_->find(map, source_buffer_,
                                 max_correspondence_distance_,
                                 correspondences_buffer_);

    inputs_buffer_.clear();
    map_points_buffer_.clear();
    for (const auto &[scan_point, map_point] : correspondences_buffer_) {
      inputs_buffer_.emplace_back(scan_point.x, scan_point.y, scan_point.z);
      map_points_buffer_.emplace_back(map_point.x, map_point.y, map_point.z);
    }

    if (map_points_buffer_.empty()) {
      logger_->warn(
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    delta.setZero();
    lm.clearCosts();
    lm.addCost(std::make_shared<
               moptim::NumericalCostForwardEuler<Point3Distance, double>>(
        inputs_buffer_[0].data(), map_points_buffer_[0].data(),
        map_points_buffer_.size(), 3, 3, 6));
    const auto result = lm.optimize(delta.data());

    const auto delta_T = se3Exp(delta);
    transformCloud(delta_T, source_buffer_);
    total_T = delta_T * total_T;

    logger_->debug("Registration Iteration: {}/{}", i + 1,
                   num_registration_iterations_);

    if (result.status == moptim::Status::SMALL_DELTA) {
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
