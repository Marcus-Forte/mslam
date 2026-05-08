#pragma once

#include "slam/Transform.hh"
#include <Eigen/Dense>

namespace mslam {
/**
 * @brief 2D Point distance model
 *
 */
struct Point2Distance {
  void setState(const double *x) { transform_ = toAffine(x[0], x[1], x[2]); }

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector2d> target{measurement};
    Eigen::Map<const Eigen::Vector2d> source{input};
    Eigen::Map<Eigen::Vector2d> transformed_point{f_x};

    transformed_point = target - transform_ * source;
  }

  Eigen::Affine2d transform_;
};

/**
 * @brief 3D Point distance model
 *
 */
struct Point3Distance {
  void setState(const double *x) {
    transform_ = toAffine(x[0], x[1], x[2], x[3], x[4], x[5]);
  }

  void residual(const double * /*x*/, const double *input,
                const double *measurement, double *f_x) const {
    Eigen::Map<const Eigen::Vector3d> target{measurement};
    Eigen::Map<const Eigen::Vector3d> source{input};
    Eigen::Map<Eigen::Vector3d> transformed_point{f_x};

    transformed_point = target - transform_ * source;
  }

  Eigen::Affine3d transform_;
};
} // namespace mslam