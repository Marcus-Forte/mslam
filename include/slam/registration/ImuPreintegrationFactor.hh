#pragma once

#include "moptim/PlusOperations/SE3.h"
#include "moptim/PlusOperations/SE3xEuclideanPlusOperator.h"
#include "slam/ImuPreintegration.hh"
#include <Eigen/Dense>

namespace mslam {

/**
 * @brief NumericalModel for IMU preintegration residuals (15-DOF).
 *
 * The optimizer works on a DELTA vector (initialized to zero). This model
 * composes the delta with the current state estimate to produce the absolute
 * state_j before computing the preintegration residual.
 *
 * State layout: [se3(6), velocity(3), gyro_bias(3), accel_bias(3)]
 *
 * Construction: provide the current state estimate and preintegrator.
 * Input (held constant): previous state in the same layout.
 * Observation: unused (zeros).
 */
class ImuPreintegrationFactor {
public:
  ImuPreintegrationFactor(const ImuPreintegrator &preintegrator,
                          const Eigen::Matrix<double, 15, 1> &current_state,
                          double weight = 1.0)
      : preintegrator_(preintegrator), current_state_(current_state),
        sqrt_info_(weight * preintegrator.sqrtInformation()) {}

  void setState(const double * /*x*/) {}

  void residual(const double *x, const double *input,
                const double * /*observation*/, double *res) const {
    // x is the DELTA being optimized (starts at zero)
    // Compose with current state to get absolute state_j
    Eigen::Matrix<double, 15, 1> absolute_state;
    moptim::SE3xEuclideanPlusOperator<double>::plus(current_state_.data(), x,
                                                    absolute_state.data(), 15);

    const ImuState state_i = stateFromLayout(input);
    const ImuState state_j = stateFromLayout(absolute_state.data());

    const Eigen::Matrix<double, 15, 1> raw_residual =
        preintegrator_.residual(state_i, state_j);

    Eigen::Map<Eigen::Matrix<double, 15, 1>> res_map(res);
    res_map = sqrt_info_ * raw_residual;
  }

private:
  static ImuState stateFromLayout(const double *x) {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(x);
    const Eigen::Affine3d T = moptim::se3Exp(xi);

    ImuState state;
    state.position = T.translation();
    state.rotation = T.linear();
    state.velocity = Eigen::Map<const Eigen::Vector3d>(x + 6);
    state.gyro_bias = Eigen::Map<const Eigen::Vector3d>(x + 9);
    state.accel_bias = Eigen::Map<const Eigen::Vector3d>(x + 12);
    return state;
  }

  const ImuPreintegrator &preintegrator_;
  Eigen::Matrix<double, 15, 1> current_state_;
  Eigen::Matrix<double, 15, 15> sqrt_info_;
};

} // namespace mslam
