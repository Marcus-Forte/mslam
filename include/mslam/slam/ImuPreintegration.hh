#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

namespace mslam {

/**
 * @brief Full navigation state: position, rotation, velocity, and IMU biases.
 *
 * This is the 15-DOF state optimized jointly with scan-matching and IMU
 * preintegration residuals. The parametrization for optimization is:
 *   [tx, ty, tz, rx, ry, rz, vx, vy, vz, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z]
 * where (tx..rz) form an se(3) tangent vector and the rest are Euclidean.
 */
struct ImuState {
  static constexpr int kDim = 15;
  static constexpr int kPosIdx = 0;
  static constexpr int kRotIdx = 3;
  static constexpr int kVelIdx = 6;
  static constexpr int kGyroBiasIdx = 9;
  static constexpr int kAccelBiasIdx = 12;

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();

  /// Pack state into a 15-element vector (rotation as angle-axis).
  Eigen::Matrix<double, 15, 1> toVector() const;

  /// Reconstruct state from a 15-element vector.
  static ImuState fromVector(const Eigen::Matrix<double, 15, 1> &x);
};

/**
 * @brief Single raw IMU measurement (body frame).
 */
struct ImuMeasurement {
  double timestamp_s;
  Eigen::Vector3d gyroscope;     // rad/s
  Eigen::Vector3d accelerometer; // m/s²
};

/**
 * @brief Noise parameters for IMU preintegration covariance propagation.
 */
struct ImuNoiseParams {
  double gyro_noise_density = 1e-3;     // rad/s/√Hz
  double accel_noise_density = 1e-2;    // m/s²/√Hz
  double gyro_bias_random_walk = 1e-4;  // rad/s²/√Hz
  double accel_bias_random_walk = 1e-3; // m/s³/√Hz
};

/**
 * @brief The preintegrated measurement between two states: what the
 * preintegration residual needs, as a plain value.
 *
 * It is copied into the IMU factor (a moptim model, i.e. SYCL device code), so
 * it holds no pointer or reference to host data, and everything that reads it
 * (preintegrationResidual() and the SO(3) helpers below) is inline and
 * device-safe: no Eigen::AngleAxis from a matrix (stableNorm() cannot be
 * compiled into a kernel), no out-of-line call.
 */
struct PreintegratedImu {
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();  ///< Linearization point.
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero(); ///< Linearization point.
  Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
  double dt = 0.0;
  Eigen::Matrix3d J_R_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_v_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_v_ba = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_p_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_p_ba = Eigen::Matrix3d::Zero();
};

namespace imu_detail {

/// exp: so(3) -> SO(3).
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d &omega) {
  const double theta = omega.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Vector3d axis = omega / theta;
  return Eigen::AngleAxisd(theta, axis).toRotationMatrix();
}

/// log: SO(3) -> so(3). Eigen::AngleAxisd(R) with its quaternion -> angle-axis
/// step written out: that step calls stableNorm() when the vector part is tiny,
/// and stableNorm()/maxCoeff() cannot be compiled into a SYCL kernel (a function
/// pointer is called in the packet min/max). Same arithmetic otherwise; for a
/// vector part below machine epsilon the plain norm replaces stableNorm().
inline Eigen::Vector3d logSO3(const Eigen::Matrix3d &R) {
  const Eigen::Quaterniond q(R);
  double n = q.vec().norm();
  if (n == 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double angle = 2.0 * std::atan2(n, std::abs(q.w()));
  if (q.w() < 0.0) {
    n = -n;
  }
  const Eigen::Vector3d axis = q.vec() / n;
  return axis * angle;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/// Right Jacobian of SO(3): Exp(phi + d) ~ Exp(phi) Exp(Jr(phi) d).
inline Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d &phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d W = skew(phi);
  if (theta < 1e-5) {
    return Eigen::Matrix3d::Identity() - 0.5 * W + (1.0 / 6.0) * W * W;
  }
  const double theta2 = theta * theta;
  return Eigen::Matrix3d::Identity() - ((1.0 - std::cos(theta)) / theta2) * W +
         ((theta - std::sin(theta)) / (theta2 * theta)) * W * W;
}

/// Inverse right Jacobian of SO(3): Log(Exp(phi) Exp(d)) ~ phi + Jr^-1(phi) d.
inline Eigen::Matrix3d rightJacobianInverseSO3(const Eigen::Vector3d &phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d W = skew(phi);
  if (theta < 1e-5) {
    return Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 12.0) * W * W;
  }
  const double theta2 = theta * theta;
  return Eigen::Matrix3d::Identity() + 0.5 * W +
         (1.0 / theta2 -
          (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta))) *
             W * W;
}

} // namespace imu_detail

/// The 15-dimensional preintegration residual between two states:
/// [delta_R_error(3), delta_v_error(3), delta_p_error(3), bias_gyro_error(3),
///  bias_accel_error(3)]. Device-safe (see PreintegratedImu).
inline Eigen::Matrix<double, 15, 1>
preintegrationResidual(const PreintegratedImu &pre, const ImuState &state_i,
                       const ImuState &state_j) {
  constexpr double kGravity = 9.80665;
  const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);

  const Eigen::Matrix3d R_i = state_i.rotation;
  const Eigen::Matrix3d R_j = state_j.rotation;
  const double dt = pre.dt;

  // First-order bias correction from linearization point
  const Eigen::Vector3d dbg = state_i.gyro_bias - pre.bias_gyro;
  const Eigen::Vector3d dba = state_i.accel_bias - pre.bias_accel;

  const Eigen::Matrix3d corrected_delta_R =
      pre.delta_R * imu_detail::expSO3(pre.J_R_bg * dbg);
  const Eigen::Vector3d corrected_delta_v =
      pre.delta_v + pre.J_v_bg * dbg + pre.J_v_ba * dba;
  const Eigen::Vector3d corrected_delta_p =
      pre.delta_p + pre.J_p_bg * dbg + pre.J_p_ba * dba;

  Eigen::Matrix<double, 15, 1> r;

  // Rotation residual: Log(delta_R_corrected^T * R_i^T * R_j)
  r.segment<3>(0) = imu_detail::logSO3(corrected_delta_R.transpose() *
                                       R_i.transpose() * R_j);

  // Velocity residual: R_i^T * (v_j - v_i - g*dt) - corrected_delta_v
  r.segment<3>(3) =
      R_i.transpose() * (state_j.velocity - state_i.velocity - gravity * dt) -
      corrected_delta_v;

  // Position residual: R_i^T * (p_j - p_i - v_i*dt - 0.5*g*dt²) -
  // corrected_delta_p
  r.segment<3>(6) =
      R_i.transpose() * (state_j.position - state_i.position -
                         state_i.velocity * dt - 0.5 * gravity * dt * dt) -
      corrected_delta_p;

  // Bias residuals (penalize drift from previous biases)
  r.segment<3>(9) = state_j.gyro_bias - state_i.gyro_bias;
  r.segment<3>(12) = state_j.accel_bias - state_i.accel_bias;

  return r;
}

/**
 * @brief IMU preintegration accumulator.
 *
 * Accumulates raw IMU measurements between two keyframes and produces
 * preintegrated rotation, velocity, and position deltas. Supports first-order
 * bias correction via stored Jacobians.
 */
class ImuPreintegrator {
public:
  explicit ImuPreintegrator(
      const ImuNoiseParams &noise,
      const Eigen::Vector3d &initial_gyro_bias = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &initial_accel_bias = Eigen::Vector3d::Zero());

  /// Integrate a single measurement with time step dt.
  void integrate(const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel,
                 double dt);

  /// Integrate from a vector of timestamped measurements.
  void integrate(const std::vector<ImuMeasurement> &measurements);

  /// Reset accumulated state (e.g. after a new keyframe).
  void reset(const Eigen::Vector3d &gyro_bias,
             const Eigen::Vector3d &accel_bias);

  /// Compute the 15-dimensional preintegration residual between two states.
  /// residual = [delta_R_error(3), delta_v_error(3), delta_p_error(3),
  ///             bias_gyro_error(3), bias_accel_error(3)]
  Eigen::Matrix<double, 15, 1> residual(const ImuState &state_i,
                                        const ImuState &state_j) const {
    return preintegrationResidual(preintegrated(), state_i, state_j);
  }

  /// The preintegrated measurement as a value (what residual() reads).
  PreintegratedImu preintegrated() const {
    PreintegratedImu pre;
    pre.bias_gyro = bias_gyro_;
    pre.bias_accel = bias_accel_;
    pre.delta_R = delta_R_;
    pre.delta_v = delta_v_;
    pre.delta_p = delta_p_;
    pre.dt = dt_sum_;
    pre.J_R_bg = J_R_bg_;
    pre.J_v_bg = J_v_bg_;
    pre.J_v_ba = J_v_ba_;
    pre.J_p_bg = J_p_bg_;
    pre.J_p_ba = J_p_ba_;
    return pre;
  }

  /// Preintegrated rotation delta (bias-corrected at linearization point).
  const Eigen::Matrix3d &deltaRotation() const { return delta_R_; }

  /// Preintegrated velocity delta.
  const Eigen::Vector3d &deltaVelocity() const { return delta_v_; }

  /// Preintegrated position delta.
  const Eigen::Vector3d &deltaPosition() const { return delta_p_; }

  /// Total integration time.
  double deltaTime() const { return dt_sum_; }

  /// Covariance of the preintegrated measurement (15x15).
  const Eigen::Matrix<double, 15, 15> &covariance() const {
    return covariance_;
  }

  /// Square-root information matrix for weighting residuals.
  Eigen::Matrix<double, 15, 15> sqrtInformation() const;

  /// Jacobian of preintegrated rotation w.r.t. gyro bias.
  const Eigen::Matrix3d &jacobianRotationBiasGyro() const { return J_R_bg_; }

  /// Jacobian of preintegrated velocity w.r.t. gyro bias.
  const Eigen::Matrix3d &jacobianVelocityBiasGyro() const { return J_v_bg_; }

  /// Jacobian of preintegrated velocity w.r.t. accel bias.
  const Eigen::Matrix3d &jacobianVelocityBiasAccel() const { return J_v_ba_; }

  /// Jacobian of preintegrated position w.r.t. gyro bias.
  const Eigen::Matrix3d &jacobianPositionBiasGyro() const { return J_p_bg_; }

  /// Jacobian of preintegrated position w.r.t. accel bias.
  const Eigen::Matrix3d &jacobianPositionBiasAccel() const { return J_p_ba_; }

private:
  static Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d &omega);

  ImuNoiseParams noise_;
  Eigen::Vector3d bias_gyro_;
  Eigen::Vector3d bias_accel_;

  // Preintegrated deltas
  Eigen::Matrix3d delta_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_v_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_p_ = Eigen::Vector3d::Zero();
  double dt_sum_ = 0.0;

  // Bias-correction Jacobians
  Eigen::Matrix3d J_R_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_v_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_v_ba_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_p_bg_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d J_p_ba_ = Eigen::Matrix3d::Zero();

  // Covariance (order: rotation, velocity, position, bias_gyro, bias_accel)
  Eigen::Matrix<double, 15, 15> covariance_ =
      Eigen::Matrix<double, 15, 15>::Zero();
};

} // namespace mslam
