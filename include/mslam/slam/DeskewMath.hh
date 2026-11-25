#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstddef>

namespace mslam::deskew_detail {

/// Body-frame rotation accumulated while turning at @p omega for @p dt
/// seconds (identity for a zero, negative or non-finite angle).
inline Eigen::Matrix3d rotationIncrement(const Eigen::Vector3d &omega,
                                         double dt) {
  const double angle = omega.norm() * dt;
  // !(angle > 0) rejects NaN too; angle - angle is NaN for an infinite angle.
  if (!(angle > 0.0) || !(angle - angle == 0.0)) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, omega.normalized()).toRotationMatrix();
}

/// Memoizes rotationIncrement() for the current angular velocity. Between two
/// IMU samples every point advances by the same time step (up to the
/// quantization of nanosecond timestamps in a double, so only a few distinct
/// steps occur), which makes the per-point axis-angle conversion redundant.
/// Returns exactly what rotationIncrement() would, bit for bit.
class RotationIncrementMemo {
public:
  const Eigen::Matrix3d &get(const Eigen::Vector3d &omega, double dt) {
    for (std::size_t i = 0; i < count_; ++i) {
      if (entries_[i].dt == dt) {
        return entries_[i].rotation;
      }
    }
    Entry &entry = entries_[next_];
    entry.dt = dt;
    entry.rotation = rotationIncrement(omega, dt);
    next_ = (next_ + 1) % kEntries;
    count_ = count_ + 1 < kEntries ? count_ + 1 : kEntries;
    return entry.rotation;
  }

  /// The angular velocity changed: forget everything.
  void reset() {
    count_ = 0;
    next_ = 0;
  }

private:
  static constexpr std::size_t kEntries = 4;
  struct Entry {
    double dt = 0.0;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  };
  Entry entries_[kEntries];
  std::size_t count_ = 0;
  std::size_t next_ = 0;
};

} // namespace mslam::deskew_detail
