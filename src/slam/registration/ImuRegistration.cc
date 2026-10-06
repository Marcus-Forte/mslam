#include "mslam/slam/registration/ImuRegistration.hh"

#include "moptim/LevenbergMarquardt.hh"
#include "moptim/NumericalCostCentral.hh"
#include "mslam/OptimizerObserver.hh"
#include "mslam/slam/NormalEstimator.hh"
#include "mslam/slam/SE3.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/ImuPreintegrationFactor.hh"
#include "mslam/slam/registration/PointDistance.hh"

#include <Eigen/Dense>

namespace mslam {
namespace {

constexpr int kStateDim = ImuPreintegrationFactor::kStateDim;
constexpr int kExtraDim = ImuPreintegrationFactor::kExtraDim;
constexpr int kParamDim = ImuPreintegrationFactor::kParamDim;
constexpr int kResidualDim = ImuPreintegrationFactor::kResidualDim;

} // namespace

SlamState ImuRegistration::Align(const SlamState & /*state*/,
                                 const IMap & /*map*/,
                                 const PointCloud & /*scan*/) {
  throw std::runtime_error(
      "Not implemented: use the IMU Align variant with preintegration.");
}

SlamState ImuRegistration::Align(const SlamState &current, const IMap &map,
                                 const PointCloud &scan,
                                 const SlamState &prev_state,
                                 const ImuPreintegrator &preintegrator) {
  static constexpr int k_maxSmallDeltaHits = 3;
  // Low weight so scan matching dominates pose.
  static constexpr double kImuWeight = 0.01;

  NormalEstimator normal_estimator(map);
  auto total_T = toAffine(current.position.x(), current.position.y(),
                          current.position.z(), current.rotation.x(),
                          current.rotation.y(), current.rotation.z());

  source_buffer_ = scan;
  transformCloud(total_T, source_buffer_);

  // Linearization point of state_j: [se3_pose(6), velocity(3), gyro_bias(3),
  // accel_bias(3)]
  Eigen::Matrix<double, kStateDim, 1> opt_state =
      Eigen::Matrix<double, kStateDim, 1>::Zero();
  opt_state.head<6>() = se3Log(total_T);
  opt_state.segment<3>(6) = prev_state.velocity;
  opt_state.segment<3>(9) = prev_state.gyro_bias;
  opt_state.segment<3>(12) = prev_state.accel_bias;

  // Previous state, same layout. Its pose is fixed; its velocity/biases are
  // the prior mean for state_i's estimated velocity/biases.
  Eigen::Affine3d prev_T = Eigen::Affine3d::Identity();
  prev_T.linear() = toAffine(0, 0, 0, prev_state.rotation.x(),
                             prev_state.rotation.y(), prev_state.rotation.z())
                        .linear();
  prev_T.translation() = prev_state.position;
  Eigen::Matrix<double, kStateDim, 1> prev_state_vec;
  prev_state_vec.head<6>() = se3Log(prev_T);
  prev_state_vec.segment<3>(6) = prev_state.velocity;
  prev_state_vec.segment<3>(9) = prev_state.gyro_bias;
  prev_state_vec.segment<3>(12) = prev_state.accel_bias;

  // Linearization point of state_i's velocity and biases.
  Eigen::Matrix<double, kExtraDim, 1> extras_i =
      prev_state_vec.tail<kExtraDim>();

  // Observation placeholder (unused by ImuPreintegrationFactor)
  const Eigen::Matrix<double, kResidualDim, 1> zeros =
      Eigen::Matrix<double, kResidualDim, 1>::Zero();

  int small_delta_hits = 0;

  inputs_buffer_.reserve(scan.size());
  map_points_buffer_.reserve(scan.size());

  Eigen::Matrix<double, kParamDim, 1> delta =
      Eigen::Matrix<double, kParamDim, 1>::Zero();
  moptim::LevenbergMarquardt<double> lm(kParamDim);
  lm.setMaxIterations(num_optimizer_iterations_);
  // auto loss_function =
  //     std::make_shared<moptim::GemanMcClureLoss<double>>(geman_mcclure_kappa_);
  OptimizerObserver<double> observer(logger_);
  lm.setObserver(&observer);

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
      logger_->warn("ImuRegistration found no correspondences.");
      break;
    }

    lm.clearCosts();

    // Scan-matching cost (point-to-plane, only uses pose DOFs 0-5)
    auto scan_matching_cost = std::make_shared<
        moptim::NumericalCostCentral<Point3PlaneDistance, double>>(
        inputs_buffer_[0].data(), map_points_buffer_[0].data(),
        map_points_buffer_.size(), 6, 3, kParamDim, Point3PlaneDistance{},
        /*active_param_dim=*/6);
    // scan_matching_cost->setLossFunction(loss_function);
    lm.addCost(scan_matching_cost);

    // IMU preintegration factor between the fixed previous pose and the
    // estimated state_j, with state_i's velocity/biases estimated as well.
    lm.addCost(std::make_shared<
               moptim::NumericalCostCentral<ImuPreintegrationFactor, double>>(
        prev_state_vec.data(), zeros.data(), 1, kStateDim, kResidualDim,
        kParamDim,
        ImuPreintegrationFactor(preintegrator, opt_state, extras_i,
                                kImuWeight)));

    delta.setZero();
    const auto result = lm.optimize(delta.data());

    // The optimizer variable is an increment around the linearization point:
    // a left-multiplied se3 increment for the pose, additive for the rest.
    Eigen::Matrix<double, kStateDim, 1> new_state;
    SE3xEuclideanPlusOperator<double>::plus(opt_state.data(), delta.data(),
                                            new_state.data(), kStateDim);
    opt_state = new_state;
    extras_i += delta.tail<kExtraDim>();

    // Update source cloud with new pose for next correspondence search
    const auto new_T = se3Exp(opt_state.head<6>());
    // Re-transform from original scan
    source_buffer_ = scan;
    transformCloud(new_T, source_buffer_);
    total_T = new_T;

    if (result.status == moptim::Status::SMALL_DELTA) {
      if (++small_delta_hits > k_maxSmallDeltaHits)
        break;
    }
  }

  const auto t = total_T.translation();
  const auto &R = total_T.linear();

  SlamState result;
  result.position = t;
  result.rotation = {std::atan2(-R(1, 2), R(2, 2)),
                     std::asin(std::clamp(R(0, 2), -1.0, 1.0)),
                     std::atan2(-R(0, 1), R(0, 0))};
  result.velocity = opt_state.segment<3>(6);
  result.gyro_bias = opt_state.segment<3>(9);
  result.accel_bias = opt_state.segment<3>(12);

  return result;
}

} // namespace mslam
