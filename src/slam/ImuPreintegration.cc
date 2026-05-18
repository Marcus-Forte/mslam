#include "slam/ImuPreintegration.hh"

#include <Eigen/Eigenvalues>
#include <cmath>

namespace {

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

} // namespace

namespace mslam {

// ─── ImuState
// ─────────────────────────────────────────────────────────────────

Eigen::Matrix<double, 15, 1> ImuState::toVector() const {
  Eigen::Matrix<double, 15, 1> x;
  x.segment<3>(kPosIdx) = position;
  const Eigen::AngleAxisd aa(rotation);
  x.segment<3>(kRotIdx) = aa.axis() * aa.angle();
  x.segment<3>(kVelIdx) = velocity;
  x.segment<3>(kGyroBiasIdx) = gyro_bias;
  x.segment<3>(kAccelBiasIdx) = accel_bias;
  return x;
}

ImuState ImuState::fromVector(const Eigen::Matrix<double, 15, 1> &x) {
  ImuState state;
  state.position = x.segment<3>(kPosIdx);
  const Eigen::Vector3d omega = x.segment<3>(kRotIdx);
  const double theta = omega.norm();
  if (theta > 1e-10) {
    state.rotation = Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
  } else {
    state.rotation = Eigen::Matrix3d::Identity();
  }
  state.velocity = x.segment<3>(kVelIdx);
  state.gyro_bias = x.segment<3>(kGyroBiasIdx);
  state.accel_bias = x.segment<3>(kAccelBiasIdx);
  return state;
}

// ─── SO(3) helpers
// ────────────────────────────────────────────────────────────

Eigen::Matrix3d ImuPreintegrator::expSO3(const Eigen::Vector3d &omega) {
  const double theta = omega.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Vector3d axis = omega / theta;
  return Eigen::AngleAxisd(theta, axis).toRotationMatrix();
}

Eigen::Vector3d ImuPreintegrator::logSO3(const Eigen::Matrix3d &R) {
  const Eigen::AngleAxisd aa(R);
  return aa.axis() * aa.angle();
}

Eigen::Matrix3d
ImuPreintegrator::rightJacobianSO3(const Eigen::Vector3d &omega) {
  const double theta = omega.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Matrix3d omega_hat =
      (Eigen::Matrix3d() << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(),
       -omega.y(), omega.x(), 0)
          .finished();
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  return Eigen::Matrix3d::Identity() -
         ((1.0 - c) / (theta * theta)) * omega_hat +
         ((theta - s) / (theta * theta * theta)) * omega_hat * omega_hat;
}

// ─── ImuPreintegrator
// ─────────────────────────────────────────────────────────

ImuPreintegrator::ImuPreintegrator(const ImuNoiseParams &noise,
                                   const Eigen::Vector3d &initial_gyro_bias,
                                   const Eigen::Vector3d &initial_accel_bias)
    : noise_(noise), bias_gyro_(initial_gyro_bias),
      bias_accel_(initial_accel_bias) {}

void ImuPreintegrator::reset(const Eigen::Vector3d &gyro_bias,
                             const Eigen::Vector3d &accel_bias) {
  bias_gyro_ = gyro_bias;
  bias_accel_ = accel_bias;
  delta_R_ = Eigen::Matrix3d::Identity();
  delta_v_ = Eigen::Vector3d::Zero();
  delta_p_ = Eigen::Vector3d::Zero();
  dt_sum_ = 0.0;
  J_R_bg_.setZero();
  J_v_bg_.setZero();
  J_v_ba_.setZero();
  J_p_bg_.setZero();
  J_p_ba_.setZero();
  covariance_.setZero();
}

void ImuPreintegrator::integrate(const Eigen::Vector3d &gyro,
                                 const Eigen::Vector3d &accel, double dt) {
  // Bias-corrected measurements
  const Eigen::Vector3d omega = gyro - bias_gyro_;
  const Eigen::Vector3d acc = accel - bias_accel_;

  // Incremental rotation
  const Eigen::Vector3d omega_dt = omega * dt;
  const Eigen::Matrix3d dR = expSO3(omega_dt);
  const Eigen::Matrix3d Jr = rightJacobianSO3(omega_dt);

  // ── Covariance propagation ──────────────────────────────────────────────
  // State order in covariance: [dR(3), dv(3), dp(3), bg(3), ba(3)]
  // Discrete-time noise covariance
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
  // dR row
  F.block<3, 3>(0, 0) = dR.transpose();
  F.block<3, 3>(0, 9) = -Jr * dt;
  // dv row
  F.block<3, 3>(3, 0) = -delta_R_ * skewSymmetric(acc) * dt;
  F.block<3, 3>(3, 12) = -delta_R_ * dt;
  // dp row
  F.block<3, 3>(6, 0) = -0.5 * delta_R_ * skewSymmetric(acc) * dt * dt;
  F.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(6, 12) = -0.5 * delta_R_ * dt * dt;

  Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
  const double ng2 = noise_.gyro_noise_density * noise_.gyro_noise_density;
  const double na2 = noise_.accel_noise_density * noise_.accel_noise_density;
  const double nbg2 =
      noise_.gyro_bias_random_walk * noise_.gyro_bias_random_walk;
  const double nba2 =
      noise_.accel_bias_random_walk * noise_.accel_bias_random_walk;
  Q.block<3, 3>(0, 0) = ng2 * dt * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(3, 3) = na2 * dt * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(6, 6) =
      na2 * dt * Eigen::Matrix3d::Identity(); // position noise from accel
  Q.block<3, 3>(9, 9) = nbg2 * dt * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(12, 12) = nba2 * dt * Eigen::Matrix3d::Identity();

  covariance_ = F * covariance_ * F.transpose() + Q;

  // ── Bias-correction Jacobians ───────────────────────────────────────────
  // J_p_bg update must use pre-update J_v_bg
  J_p_bg_ +=
      J_v_bg_ * dt - 0.5 * delta_R_ * skewSymmetric(acc) * J_R_bg_ * dt * dt;
  J_p_ba_ += J_v_ba_ * dt - 0.5 * delta_R_ * dt * dt;
  J_v_bg_ -= delta_R_ * skewSymmetric(acc) * J_R_bg_ * dt;
  J_v_ba_ -= delta_R_ * dt;
  J_R_bg_ = dR.transpose() * J_R_bg_ - Jr * dt;

  // ── Preintegrated deltas ────────────────────────────────────────────────
  delta_p_ += delta_v_ * dt + 0.5 * delta_R_ * acc * dt * dt;
  delta_v_ += delta_R_ * acc * dt;
  delta_R_ = delta_R_ * dR;
  dt_sum_ += dt;
}

void ImuPreintegrator::integrate(
    const std::vector<ImuMeasurement> &measurements) {
  for (size_t i = 1; i < measurements.size(); ++i) {
    const double dt =
        measurements[i].timestamp_s - measurements[i - 1].timestamp_s;
    if (dt <= 0.0) {
      continue;
    }
    // Mid-point integration
    const Eigen::Vector3d gyro_mid =
        0.5 * (measurements[i - 1].gyroscope + measurements[i].gyroscope);
    const Eigen::Vector3d accel_mid = 0.5 * (measurements[i - 1].accelerometer +
                                             measurements[i].accelerometer);
    integrate(gyro_mid, accel_mid, dt);
  }
}

Eigen::Matrix<double, 15, 1>
ImuPreintegrator::residual(const ImuState &state_i,
                           const ImuState &state_j) const {
  static constexpr double kGravity = 9.80665;
  const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);

  const Eigen::Matrix3d R_i = state_i.rotation;
  const Eigen::Matrix3d R_j = state_j.rotation;
  const double dt = dt_sum_;

  // First-order bias correction from linearization point
  const Eigen::Vector3d dbg = state_i.gyro_bias - bias_gyro_;
  const Eigen::Vector3d dba = state_i.accel_bias - bias_accel_;

  const Eigen::Matrix3d corrected_delta_R = delta_R_ * expSO3(J_R_bg_ * dbg);
  const Eigen::Vector3d corrected_delta_v =
      delta_v_ + J_v_bg_ * dbg + J_v_ba_ * dba;
  const Eigen::Vector3d corrected_delta_p =
      delta_p_ + J_p_bg_ * dbg + J_p_ba_ * dba;

  Eigen::Matrix<double, 15, 1> r;

  // Rotation residual: Log(delta_R_corrected^T * R_i^T * R_j)
  r.segment<3>(0) =
      logSO3(corrected_delta_R.transpose() * R_i.transpose() * R_j);

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

Eigen::Matrix<double, 15, 15> ImuPreintegrator::sqrtInformation() const {
  // Regularize covariance for numerical stability
  Eigen::Matrix<double, 15, 15> cov = covariance_;
  cov.diagonal().array() += 1e-10;

  // Compute information = cov^{-1}, then matrix square root
  const Eigen::Matrix<double, 15, 15> info = cov.inverse();

  // Use LLT decomposition: info = L * L^T, sqrt_info = L^T
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(info);
  if (llt.info() != Eigen::Success) {
    // Fallback: use diagonal sqrt of information
    return info.diagonal().cwiseSqrt().asDiagonal();
  }
  return llt.matrixU();
}

} // namespace mslam
