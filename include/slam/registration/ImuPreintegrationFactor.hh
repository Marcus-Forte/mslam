#pragma once

#include "slam/ImuPreintegration.hh"
#include "slam/SE3.hh"
#include <Eigen/Dense>

namespace mslam {

/// 1-sigma of the prior on state_i's velocity and biases (before weighting).
struct ImuPriorSigma {
  double velocity = 0.05;   // m/s
  double gyro_bias = 3e-4;  // rad/s
  double accel_bias = 1e-2; // m/s^2
};

/**
 * @brief NumericalModel for the two-state IMU preintegration residual.
 *
 * Both ends of the preintegration interval are estimated:
 *  - state_j (the new state): pose as se(3) + velocity + gyro/accel bias.
 *  - state_i (the previous state): velocity + gyro/accel bias. Its pose is
 *    held fixed (it is the gauge and is anchored by scan matching).
 *
 * Estimating the velocity and biases of state_i is what makes them observable:
 * the rotation/position residuals then constrain the bias used to correct the
 * preintegrated deltas, instead of the bias merely being copied from the
 * previous state.
 *
 * The optimizer works on a DELTA vector (initialized to zero) with layout
 *   [delta_j(15) = se3(6), velocity(3), gyro_bias(3), accel_bias(3);
 *    delta_i(9)  = velocity(3), gyro_bias(3), accel_bias(3)]
 * composed with the linearization points given at construction.
 *
 * Input (held constant): previous state, layout [se3(6), v(3), bg(3), ba(3)].
 * Its velocity/biases also act as the prior mean for state_i; the prior stands
 * in for the history before state_i, which is not modelled.
 * Observation: unused (zeros, kResidualDim entries).
 */
class ImuPreintegrationFactor {
public:
  static constexpr int kStateDim = 15;
  static constexpr int kExtraDim = 9;
  static constexpr int kParamDim = kStateDim + kExtraDim;
  static constexpr int kResidualDim = 15 + kExtraDim;

  /// @param current_state_j  Linearization point of state_j
  /// @param current_extras_i Linearization point of state_i [v, bg, ba]
  ImuPreintegrationFactor(const ImuPreintegrator &preintegrator,
                          const Eigen::Matrix<double, 15, 1> &current_state_j,
                          const Eigen::Matrix<double, 9, 1> &current_extras_i,
                          double weight = 1.0,
                          const ImuPriorSigma &sigma = ImuPriorSigma{})
      : preintegrator_(preintegrator), current_state_j_(current_state_j),
        current_extras_i_(current_extras_i),
        sqrt_info_(weight * preintegrator.sqrtInformation()) {
    prior_sqrt_info_ << Eigen::Vector3d::Constant(1.0 / sigma.velocity),
        Eigen::Vector3d::Constant(1.0 / sigma.gyro_bias),
        Eigen::Vector3d::Constant(1.0 / sigma.accel_bias);
    prior_sqrt_info_ *= weight;
  }

  void setState(const double * /*x*/) {}

  void residual(const double *x, const double *input,
                const double * /*observation*/, double *res) const {
    Eigen::Matrix<double, 15, 1> state_j_vec;
    SE3xEuclideanPlusOperator<double>::plus(current_state_j_.data(), x,
                                            state_j_vec.data(), kStateDim);

    const Eigen::Map<const Eigen::Matrix<double, 9, 1>> delta_i(x + kStateDim);
    const Eigen::Matrix<double, 9, 1> extras_i = current_extras_i_ + delta_i;

    // state_i: fixed pose from `input`, estimated velocity and biases.
    Eigen::Matrix<double, 15, 1> state_i_vec;
    state_i_vec.head<6>() =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(input);
    state_i_vec.tail<9>() = extras_i;

    const ImuState state_i = stateFromLayout(state_i_vec.data());
    const ImuState state_j = stateFromLayout(state_j_vec.data());

    Eigen::Map<Eigen::Matrix<double, kResidualDim, 1>> res_map(res);
    res_map.head<15>() = sqrt_info_ * preintegrator_.residual(state_i, state_j);

    const Eigen::Map<const Eigen::Matrix<double, 9, 1>> prior_mean(input + 6);
    res_map.tail<kExtraDim>() =
        prior_sqrt_info_.cwiseProduct(extras_i - prior_mean);
  }

private:
  static ImuState stateFromLayout(const double *x) {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(x);
    const Eigen::Affine3d T = se3Exp(xi);

    ImuState state;
    state.position = T.translation();
    state.rotation = T.linear();
    state.velocity = Eigen::Map<const Eigen::Vector3d>(x + 6);
    state.gyro_bias = Eigen::Map<const Eigen::Vector3d>(x + 9);
    state.accel_bias = Eigen::Map<const Eigen::Vector3d>(x + 12);
    return state;
  }

  const ImuPreintegrator &preintegrator_;
  Eigen::Matrix<double, 15, 1> current_state_j_;
  Eigen::Matrix<double, 9, 1> current_extras_i_;
  Eigen::Matrix<double, 15, 15> sqrt_info_;
  Eigen::Matrix<double, 9, 1> prior_sqrt_info_;
};

} // namespace mslam
