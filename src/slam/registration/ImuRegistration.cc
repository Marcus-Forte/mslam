#include "slam/registration/ImuRegistration.hh"

#include "moptim/LevenbergMarquardt.h"
#include "moptim/NumericalCostCentral.h"
#include "moptim/NumericalCostForwardEuler.h"
#include "moptim/PlusOperations/SE3PlusOperator.h"
#include "moptim/PlusOperations/SE3xEuclideanPlusOperator.h"
#include "slam/NormalEstimator.hh"
#include "slam/Transform.hh"
#include "slam/registration/ImuPreintegrationFactor.hh"

#include <Eigen/Dense>

namespace mslam {
namespace {

/// Point-to-plane model for 15-DOF state.
/// Uses only x[0..5] (se3 exponential map) for geometry.
/// Velocity/bias dimensions produce zero Jacobian columns naturally via
/// numerical differentiation.
struct Point3PlaneDistanceSE3 {
  void setState(const double *x) {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(x);
    transform_ = moptim::se3Exp(xi);
  }

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<const Eigen::Vector3d> normal{input + 3};
    Eigen::Map<const Eigen::Vector3d> target{measurement};
    Eigen::Map<Eigen::Vector3d> res{f_x};

    const double signed_distance = normal.dot(target - transform_ * source);
    res = normal * signed_distance;
  }

  Eigen::Affine3d transform_ = Eigen::Affine3d::Identity();
};

} // namespace

using SE3Plus = moptim::SE3PlusOperator<double>;
using SE3xEucPlus = moptim::SE3xEuclideanPlusOperator<double>;

SlamState ImuRegistration::Align(const SlamState &state, const IMap &map,
                                 const PointCloud &scan) {

  throw std::runtime_error(
      "Not implemented: use the 15-DOF Align variant with IMU preintegration.");
}

SlamState ImuRegistration::Align(const SlamState &current, const IMap &map,
                                 const PointCloud &scan,
                                 const SlamState &prev_state,
                                 const ImuPreintegrator &preintegrator) {
  static constexpr int k_maxSmallDeltaHits = 3;
  // Low weight so scan matching dominates pose. Velocity is estimated
  // kinematically after optimization (not from the IMU factor).
  static constexpr double kImuWeight = 0.01;

  NormalEstimator normal_estimator(map);
  auto total_T = toAffine(current.position.x(), current.position.y(),
                          current.position.z(), current.rotation.x(),
                          current.rotation.y(), current.rotation.z());

  source_buffer_ = scan;
  transformCloud(total_T, source_buffer_);

  // Build initial 15-DOF state: [se3_pose(6), velocity(3), gyro_bias(3),
  // accel_bias(3)]
  Eigen::Matrix<double, 15, 1> opt_state = Eigen::Matrix<double, 15, 1>::Zero();
  opt_state.head<6>() = moptim::se3Log(total_T);
  opt_state.segment<3>(6) = prev_state.velocity;
  opt_state.segment<3>(9) = prev_state.gyro_bias;
  opt_state.segment<3>(12) = prev_state.accel_bias;

  // Previous state as input to IMU factor — same layout as optimizer state:
  // [se3(6), velocity(3), gyro_bias(3), accel_bias(3)]
  Eigen::Affine3d prev_T = Eigen::Affine3d::Identity();
  prev_T.linear() = toAffine(0, 0, 0, prev_state.rotation.x(),
                             prev_state.rotation.y(), prev_state.rotation.z())
                        .linear();
  prev_T.translation() = prev_state.position;
  Eigen::Matrix<double, 15, 1> prev_state_vec;
  prev_state_vec.head<6>() = moptim::se3Log(prev_T);
  prev_state_vec.segment<3>(6) = prev_state.velocity;
  prev_state_vec.segment<3>(9) = prev_state.gyro_bias;
  prev_state_vec.segment<3>(12) = prev_state.accel_bias;
  // Observation placeholder (unused by ImuPreintegrationFactor)
  const Eigen::Matrix<double, 15, 1> zeros =
      Eigen::Matrix<double, 15, 1>::Zero();

  int small_delta_hits = 0;

  inputs_buffer_.reserve(scan.size());
  map_points_buffer_.reserve(scan.size());

  Eigen::Matrix<double, 15, 1> delta = Eigen::Matrix<double, 15, 1>::Zero();
  moptim::LevenbergMarquardt<double, SE3xEucPlus> lm(15, logger_);
  lm.setMaxIterations(num_optimizer_iterations_);

  for (int i = 0; i < num_registration_iterations_; ++i) {
    correspondence_finder_->find(map, source_buffer_,
                                 max_correspondence_distance_,
                                 correspondences_buffer_);

    inputs_buffer_.clear();
    map_points_buffer_.clear();
    for (const auto &[scan_point, map_point] : correspondences_buffer_) {
      const auto normal = normal_estimator.estimate(map_point);
      if (!normal.has_value())
        continue;
      inputs_buffer_.emplace_back(scan_point.x, scan_point.y, scan_point.z,
                                  normal->x(), normal->y(), normal->z());
      map_points_buffer_.emplace_back(map_point.x, map_point.y, map_point.z);
    }

    if (map_points_buffer_.empty()) {
      logger_->log(ILog::Level::WARNING,
                   "ImuRegistration (15-DOF) found no correspondences.");
      break;
    }

    lm.clearCosts();

    // Scan-matching cost (point-to-plane, only uses pose DOFs 0-5)
    lm.addCost(
        std::make_shared<moptim::NumericalCostCentral<Point3PlaneDistanceSE3,
                                                      double, SE3xEucPlus>>(
            inputs_buffer_[0].data(), map_points_buffer_[0].data(),
            map_points_buffer_.size(), 6, 3, 15));

    // IMU preintegration factor (constrains all 15 DOFs)
    // Pass current state so the factor can compose delta with it
    lm.addCost(
        std::make_shared<moptim::NumericalCostCentral<ImuPreintegrationFactor,
                                                      double, SE3xEucPlus>>(
            prev_state_vec.data(), zeros.data(), 1, 15, 15, 15,
            ImuPreintegrationFactor(preintegrator, opt_state, kImuWeight)));

    delta.setZero();
    const auto status = lm.optimize(delta.data());

    // Apply delta via plus operator
    Eigen::Matrix<double, 15, 1> new_state;
    SE3xEucPlus::plus(opt_state.data(), delta.data(), new_state.data(), 15);
    opt_state = new_state;

    // Update source cloud with new pose for next correspondence search
    const auto new_T = moptim::se3Exp(opt_state.head<6>());
    // Re-transform from original scan
    source_buffer_ = scan;
    transformCloud(new_T, source_buffer_);
    total_T = new_T;

    if (status == moptim::Status::SMALL_DELTA) {
      if (++small_delta_hits > k_maxSmallDeltaHits)
        break;
    }
  }

  // Extract results
  const auto t = total_T.translation();
  const auto &R = total_T.linear();

  SlamState result;
  result.position = t;
  result.rotation = {std::atan2(-R(1, 2), R(2, 2)),
                     std::asin(std::clamp(R(0, 2), -1.0, 1.0)),
                     std::atan2(-R(0, 1), R(0, 0))};

  // Velocity is not directly observable in a single-frame optimization:
  // the IMU velocity residual propagates v_i forward but the position residual
  // (which constrains p_j given v_i) does not involve v_j. Recompute velocity
  // from the trusted position change (scan-matched) and preintegrated data.
  const double dt = preintegrator.deltaTime();
  if (dt > 1e-6) {
    // From: p_j = p_i + v_i*dt + 0.5*g*dt² + R_i*delta_p
    // =>    v_i = (p_j - p_i - 0.5*g*dt² - R_i*delta_p) / dt
    // Then: v_j = v_i + g*dt + R_i*delta_v
    static constexpr double kGravity = 9.80665;
    const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);
    const Eigen::Matrix3d R_i = prev_T.linear();
    const Eigen::Vector3d v_i_kinematic =
        (t - prev_state.position - 0.5 * gravity * dt * dt -
         R_i * preintegrator.deltaPosition()) /
        dt;
    result.velocity =
        v_i_kinematic + gravity * dt + R_i * preintegrator.deltaVelocity();
  } else {
    result.velocity = prev_state.velocity;
  }

  result.gyro_bias = opt_state.segment<3>(9);
  result.accel_bias = opt_state.segment<3>(12);

  return result;
}

} // namespace mslam
