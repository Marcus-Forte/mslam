#pragma once

#include <Eigen/Dense>
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
                                        const ImuState &state_j) const;

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
  static Eigen::Matrix3d expSO3(const Eigen::Vector3d &omega);
  static Eigen::Vector3d logSO3(const Eigen::Matrix3d &R);
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
