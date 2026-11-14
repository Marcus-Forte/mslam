#include "mslam/slam/registration/ImuRegistration.hh"

#include "moptim/AnalyticalCost.hh"
#include "moptim/LevenbergMarquardt.hh"
#include "moptim/NumericalCostCentral.hh"
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

ImuRegistration::ImuRegistration(
    int num_registration_iterations, int num_optimizer_iterations,
    float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder)
    : icp_(IcpParameters{num_registration_iterations, num_optimizer_iterations,
                         max_correspondence_distance},
           std::move(logger), std::move(correspondence_finder)) {}

SlamState ImuRegistration::Align(const SlamState &current, const IMap &map,
                                 const PointCloud &scan,
                                 const SlamState &prev_state,
                                 const ImuPreintegrator &preintegrator) {
  static constexpr int k_maxSmallDeltaHits = 3;
  // Low weight so scan matching dominates pose.
  static constexpr double kImuWeight = 0.01;
  const auto &parameters = icp_.parameters();

  first_round_ = {};
  auto total_T = toAffine(current);
  icp_.begin(map, scan, total_T);

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
  Eigen::Matrix<double, kStateDim, 1> prev_state_vec;
  prev_state_vec.head<6>() = se3Log(toAffine(prev_state));
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

  Eigen::Matrix<double, kParamDim, 1> delta =
      Eigen::Matrix<double, kParamDim, 1>::Zero();
  moptim::LevenbergMarquardt<double> lm(kParamDim);
  lm.setMaxIterations(parameters.optimizer_iterations);
  icp_.attachObserver(lm);

  for (int i = 0; i < parameters.registration_iterations; ++i) {
    if (icp_.findPlaneConstraints(map) == 0) {
      icp_.logger().warn("ImuRegistration found no correspondences.");
      break;
    }

    lm.clearCosts();

    // Scan-matching cost (point-to-plane, only uses pose DOFs 0-5)
    lm.addCost(
        std::make_shared<moptim::AnalyticalCost<Point3PlaneDistance, double>>(
            icp_.planeInputs().front().data(), icp_.planeOffsets().data(),
            icp_.planeOffsets().size(), 6, 1, kParamDim, Point3PlaneDistance{},
            /*active_param_dim=*/6));

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
    if (i == 0) {
      first_round_ = {true, delta.head<3>().norm(), delta.segment<3>(3).norm()};
    }

    // The optimizer variable is an increment around the linearization point:
    // a left-multiplied se3 increment for the pose, additive for the rest.
    Eigen::Matrix<double, kStateDim, 1> new_state;
    SE3xEuclideanPlusOperator<double>::plus(opt_state.data(), delta.data(),
                                            new_state.data(), kStateDim);
    opt_state = new_state;
    extras_i += delta.tail<kExtraDim>();

    // Update source cloud with new pose for next correspondence search,
    // re-transforming from the original scan.
    total_T = se3Exp(opt_state.head<6>());
    icp_.resetSource(total_T);

    if (result.status == moptim::Status::SMALL_DELTA) {
      if (++small_delta_hits > k_maxSmallDeltaHits)
        break;
    }
  }

  SlamState result;
  setPose(result, total_T);
  result.velocity = opt_state.segment<3>(6);
  result.gyro_bias = opt_state.segment<3>(9);
  result.accel_bias = opt_state.segment<3>(12);

  return result;
}

} // namespace mslam
