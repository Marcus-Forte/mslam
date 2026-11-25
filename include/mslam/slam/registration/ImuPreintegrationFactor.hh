#pragma once

#include "mslam/slam/ImuPreintegration.hh"
#include "mslam/slam/SE3Manifold.hh"

#include "moptim/Manifold.hh"

#include <Eigen/Dense>
#include <memory>

namespace mslam {

/// 1-sigma of the prior on state_i's velocity and biases (before weighting).
struct ImuPriorSigma {
  double velocity = 0.05;   // m/s
  double gyro_bias = 3e-4;  // rad/s
  double accel_bias = 1e-2; // m/s^2
};

/**
 * @brief Model of the two-state IMU preintegration residual, with an analytical
 * jacobian (AnalyticalCost) and usable by finite differences (NumericalCost).
 *
 * Both ends of the preintegration interval are estimated:
 *  - state_j (the new state): pose + velocity + gyro/accel bias.
 *  - state_i (the previous state): velocity + gyro/accel bias. Its pose is
 *    held fixed (it is the gauge and is anchored by scan matching).
 *
 * Estimating the velocity and biases of state_i is what makes them observable:
 * the rotation/position residuals then constrain the bias used to correct the
 * preintegrated deltas, instead of the bias merely being copied from the
 * previous state.
 *
 * The optimizer variable x is the state itself (ambient dimension
 * kAmbientDim = 30), read directly by the residual:
 *   [pose_j(12) | v_j(3) | bg_j(3) | ba_j(3) | v_i(3) | bg_i(3) | ba_i(3)]
 * with the pose as the SE3Manifold storage ([R | t], column-major). It moves
 * on the manifold made by makeManifold(), whose step has kTangentDim = 24
 * entries:
 *   [rho, omega | dv_j, dbg_j, dba_j | dv_i, dbg_i, dba_i].
 *
 * Input (held constant, kInputDim = 21): the previous state as
 *   [pose_i(12) | v(3) | bg(3) | ba(3)].
 * Its velocity/biases also act as the prior mean for state_i; the prior stands
 * in for the history before state_i, which is not modelled.
 * Observation: unused (zeros, kResidualDim entries).
 */
class ImuPreintegrationFactor;

class ImuPreintegrationFactor {
public:
  static constexpr int kPoseStorageDim = SE3Manifold::kAmbientDim; // 12
  static constexpr int kExtrasDim = 9; // velocity, gyro bias, accel bias
  /// Entries of the optimizer variable x.
  static constexpr int kAmbientDim = kPoseStorageDim + 2 * kExtrasDim;
  /// Entries of an optimizer step.
  static constexpr int kTangentDim = SE3Manifold::kTangentDim + 2 * kExtrasDim;
  /// Entries of the constant input (the previous state).
  static constexpr int kInputDim = kPoseStorageDim + kExtrasDim;
  static constexpr int kPreintegrationResidualDim = 15;
  static constexpr int kResidualDim = kPreintegrationResidualDim + kExtrasDim;

  /// Offsets into x.
  static constexpr int kExtrasJ = kPoseStorageDim; // v_j, bg_j, ba_j
  static constexpr int kExtrasI =
      kPoseStorageDim + kExtrasDim; // v_i, bg_i, ba_i

  /// SE3Manifold x Euclidean(18): the manifold of x.
  static std::shared_ptr<const moptim::IManifold<double>>
  makeManifold(std::shared_ptr<const SE3Manifold> pose =
                   std::make_shared<SE3Manifold>()) {
    return std::make_shared<moptim::ProductManifold<double>>(
        std::vector<std::shared_ptr<const moptim::IManifold<double>>>{
            std::move(pose),
            std::make_shared<moptim::EuclideanManifold<double>>(2 *
                                                                kExtrasDim)});
  }

  ImuPreintegrationFactor(const ImuPreintegrator &preintegrator,
                          double weight = 1.0,
                          const ImuPriorSigma &sigma = ImuPriorSigma{})
      : preintegrated_(preintegrator.preintegrated()),
        sqrt_info_(weight * preintegrator.sqrtInformation()) {
    prior_sqrt_info_ << Eigen::Vector3d::Constant(1.0 / sigma.velocity),
        Eigen::Vector3d::Constant(1.0 / sigma.gyro_bias),
        Eigen::Vector3d::Constant(1.0 / sigma.accel_bias);
    prior_sqrt_info_ *= weight;
  }

  void setState(const double * /*x*/) {}

  /// d residual / d step at zero, written transposed (jac[j + kTangentDim * i]
  /// = d res[i] / d delta[j]). The step is [rho, omega | dv_j, dbg_j, dba_j |
  /// dv_i, dbg_i, dba_i] with a LEFT increment on pose_j (SE3Manifold): dR_j =
  /// [omega]x R_j, dp_j = rho + omega x p_j. Rotation residual r_R =
  /// Log(dR~^T R_i^T R_j) with dR~ = dR Exp(J_R_bg dbg): d/d omega = Jr^-1(r_R)
  /// R_j^T, d/d bg_i = -Jr^-1(r_R) E^T Jr(J_R_bg dbg) J_R_bg (E the rotation
  /// error); the velocity, position and bias rows are linear in the step.
  void jacobian(const double *x, const double *input,
                const double * /*observation*/, double *jac) const {
    using imu_detail::rightJacobianInverseSO3;
    using imu_detail::rightJacobianSO3;
    using imu_detail::skew;
    const ImuState state_i = stateFrom(input, x + kExtrasI);
    const ImuState state_j = stateFrom(x, x + kExtrasJ);
    const PreintegratedImu &pre = preintegrated_;

    const Eigen::Vector3d dbg = state_i.gyro_bias - pre.bias_gyro;
    const Eigen::Vector3d phi_bias = pre.J_R_bg * dbg;
    const Eigen::Matrix3d Ri_T = state_i.rotation.transpose();
    const Eigen::Matrix3d corrected_delta_R =
        pre.delta_R * imu_detail::expSO3(phi_bias);
    const Eigen::Matrix3d error =
        corrected_delta_R.transpose() * Ri_T * state_j.rotation;
    const Eigen::Matrix3d Jr_inv =
        rightJacobianInverseSO3(imu_detail::logSO3(error));
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // Columns of the step: rho 0, omega 3, v_j 6, bg_j 9, ba_j 12, v_i 15,
    // bg_i 18, ba_i 21.
    Eigen::Matrix<double, kPreintegrationResidualDim, kTangentDim> J =
        Eigen::Matrix<double, kPreintegrationResidualDim, kTangentDim>::Zero();
    J.block<3, 3>(0, 3) = Jr_inv * state_j.rotation.transpose();
    J.block<3, 3>(0, 18) =
        -Jr_inv * error.transpose() * rightJacobianSO3(phi_bias) * pre.J_R_bg;
    J.block<3, 3>(3, 6) = Ri_T;
    J.block<3, 3>(3, 15) = -Ri_T;
    J.block<3, 3>(3, 18) = -pre.J_v_bg;
    J.block<3, 3>(3, 21) = -pre.J_v_ba;
    J.block<3, 3>(6, 0) = Ri_T;
    J.block<3, 3>(6, 3) = -Ri_T * skew(state_j.position);
    J.block<3, 3>(6, 15) = -pre.dt * Ri_T;
    J.block<3, 3>(6, 18) = -pre.J_p_bg;
    J.block<3, 3>(6, 21) = -pre.J_p_ba;
    J.block<3, 3>(9, 9) = I;
    J.block<3, 3>(9, 18) = -I;
    J.block<3, 3>(12, 12) = I;
    J.block<3, 3>(12, 21) = -I;

    Eigen::Map<Eigen::Matrix<double, kTangentDim, kResidualDim>> jac_t(jac);
    // Coefficient-wise product: a plain product of this size takes Eigen's GEMM
    // path, which allocates.
    jac_t.leftCols<kPreintegrationResidualDim>() =
        J.transpose().lazyProduct(sqrt_info_.transpose());
    jac_t.rightCols<kExtrasDim>().setZero();
    for (int k = 0; k < kExtrasDim; ++k) {
      jac_t(kExtrasI - SE3Manifold::kAmbientDim + SE3Manifold::kTangentDim + k,
            kPreintegrationResidualDim + k) = prior_sqrt_info_[k];
    }
  }

  void residual(const double *x, const double *input,
                const double * /*observation*/, double *res) const {
    const ImuState state_i = stateFrom(input, x + kExtrasI);
    const ImuState state_j = stateFrom(x, x + kExtrasJ);

    Eigen::Map<Eigen::Matrix<double, kResidualDim, 1>> res_map(res);
    res_map.head<kPreintegrationResidualDim>() = sqrt_info_.lazyProduct(
        preintegrationResidual(preintegrated_, state_i, state_j));

    const Eigen::Map<const Eigen::Matrix<double, kExtrasDim, 1>> extras_i(
        x + kExtrasI);
    const Eigen::Map<const Eigen::Matrix<double, kExtrasDim, 1>> prior_mean(
        input + kPoseStorageDim);
    res_map.tail<kExtrasDim>() =
        prior_sqrt_info_.cwiseProduct(extras_i - prior_mean);
  }

private:
  /// State with the pose stored at @p pose_storage (12 doubles) and
  /// [v, bg, ba] at @p extras.
  static ImuState stateFrom(const double *pose_storage, const double *extras) {
    const Eigen::Map<const Eigen::Matrix<double, 3, 4>> pose(pose_storage);

    ImuState state;
    state.rotation = pose.leftCols<3>();
    state.position = pose.col(3);
    state.velocity = Eigen::Map<const Eigen::Vector3d>(extras);
    state.gyro_bias = Eigen::Map<const Eigen::Vector3d>(extras + 3);
    state.accel_bias = Eigen::Map<const Eigen::Vector3d>(extras + 6);
    return state;
  }

  // A copy, not a reference to the ImuPreintegrator: the cost copies the model,
  // so it must not point at the caller's data.
  PreintegratedImu preintegrated_;
  Eigen::Matrix<double, 15, 15> sqrt_info_;
  Eigen::Matrix<double, kExtrasDim, 1> prior_sqrt_info_;
};

} // namespace mslam
