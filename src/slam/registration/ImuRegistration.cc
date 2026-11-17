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

using Factor = ImuPreintegrationFactor;

} // namespace

ImuRegistration::ImuRegistration(
    int num_registration_iterations, int num_optimizer_iterations,
    float max_correspondence_distance, std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
    std::shared_ptr<const ParallelExecutor> executor)
    : icp_(IcpParameters{num_registration_iterations, num_optimizer_iterations,
                         max_correspondence_distance},
           std::move(logger), std::move(correspondence_finder),
           std::move(executor)),
      manifold_(Factor::makeManifold()) {}

SlamState ImuRegistration::Align(const SlamState &current, const IMap &map,
                                 const PointCloud &scan,
                                 const SlamState &prev_state,
                                 const ImuPreintegrator &preintegrator) {
  static constexpr int k_maxSmallDeltaHits = 3;
  // Low weight so scan matching dominates pose. The preintegration covariance
  // models white noise only, which understates the real IMU error (sample
  // timing, lever arm, bias drift): with 0.01 living_room_garden ended 0.98 m
  // from its start, 0.003 to 0.0001 all pass (bench_slam, 2026-11-17).
  static constexpr double kImuWeight = 0.003;
  const auto &parameters = icp_.parameters();

  first_round_ = {};
  auto total_T = toAffine(current);
  icp_.begin(map, scan, total_T);

  // The optimizer variable is the state itself (see ImuPreintegrationFactor):
  // [pose_j | v_j | bg_j | ba_j | v_i | bg_i | ba_i]. It lives across the
  // registration rounds and is the result.
  Eigen::Matrix<double, Factor::kAmbientDim, 1> state;
  poseToStorage(total_T, state.data());
  state.segment<3>(Factor::kExtrasJ) = prev_state.velocity;
  state.segment<3>(Factor::kExtrasJ + 3) = prev_state.gyro_bias;
  state.segment<3>(Factor::kExtrasJ + 6) = prev_state.accel_bias;
  state.segment<3>(Factor::kExtrasI) = prev_state.velocity;
  state.segment<3>(Factor::kExtrasI + 3) = prev_state.gyro_bias;
  state.segment<3>(Factor::kExtrasI + 6) = prev_state.accel_bias;

  // Previous state: its pose is fixed; its velocity/biases are the prior mean
  // for state_i's estimated velocity/biases.
  Eigen::Matrix<double, Factor::kInputDim, 1> prev_state_vec;
  poseToStorage(toAffine(prev_state), prev_state_vec.data());
  prev_state_vec.segment<3>(Factor::kPoseStorageDim) = prev_state.velocity;
  prev_state_vec.segment<3>(Factor::kPoseStorageDim + 3) = prev_state.gyro_bias;
  prev_state_vec.segment<3>(Factor::kPoseStorageDim + 6) =
      prev_state.accel_bias;

  // Observation placeholder (unused by ImuPreintegrationFactor)
  const Eigen::Matrix<double, Factor::kResidualDim, 1> zeros =
      Eigen::Matrix<double, Factor::kResidualDim, 1>::Zero();

  int small_delta_hits = 0;

  moptim::LevenbergMarquardt<double> lm(manifold_);
  lm.setMaxIterations(parameters.optimizer_iterations);
  icp_.attachObserver(lm);

  for (int i = 0; i < parameters.registration_iterations; ++i) {
    if (icp_.findPlaneConstraints(map) == 0) {
      icp_.logger().warn("ImuRegistration found no correspondences.");
      break;
    }

    lm.clearCosts();

    // Scan-matching cost (point-to-plane, only the pose's 6 tangent
    // dimensions). The source cloud is the scan moved by the pose at the start
    // of the round.
    const Eigen::Affine3d pose_before = total_T;
    lm.addCost(
        std::make_shared<moptim::AnalyticalCost<Point3PlaneDistance, double>>(
            icp_.planeInputs().front().data(), icp_.planeOffsets().data(),
            icp_.planeOffsets().size(), 6, 1, manifold_,
            Point3PlaneDistance(pose_before),
            /*active_param_dim=*/SE3Manifold::kTangentDim));

    // IMU preintegration factor between the fixed previous pose and the
    // estimated state_j, with state_i's velocity/biases estimated as well.
    lm.addCost(std::make_shared<
               moptim::NumericalCostCentral<ImuPreintegrationFactor, double>>(
        prev_state_vec.data(), zeros.data(), 1, Factor::kInputDim,
        Factor::kResidualDim, manifold_,
        ImuPreintegrationFactor(preintegrator, kImuWeight)));

    const auto result = lm.optimize(state.data());

    total_T = poseFromStorage(state.data());
    if (i == 0) {
      const Eigen::Matrix<double, 6, 1> step =
          se3Log(Eigen::Affine3d(total_T * pose_before.inverse()));
      first_round_ = {true, step.head<3>().norm(), step.tail<3>().norm()};
    }

    // Update source cloud with new pose for next correspondence search,
    // re-transforming from the original scan.
    icp_.resetSource(total_T);

    if (result.status == moptim::Status::SMALL_DELTA) {
      if (++small_delta_hits > k_maxSmallDeltaHits)
        break;
    }
  }

  SlamState result;
  setPose(result, total_T);
  result.velocity = state.segment<3>(Factor::kExtrasJ);
  result.gyro_bias = state.segment<3>(Factor::kExtrasJ + 3);
  result.accel_bias = state.segment<3>(Factor::kExtrasJ + 6);

  return result;
}

} // namespace mslam
