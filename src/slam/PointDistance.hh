#pragma once

#include "IModel.hh"
#include "slam/Transform.hh"
#include <Eigen/Dense>

namespace mslam {
/**
 * @brief 2D Point distance model
 *
 */
struct Point2Distance : public IModel {
  void setup(const double *x) final { transform_ = toAffine(x[0], x[1], x[2]); }

  void f(const double *input, const double *measurement, double *f_x) final {
    Eigen::Map<const Eigen::Vector2d> target{input};
    Eigen::Map<const Eigen::Vector2d> source{measurement};
    Eigen::Map<Eigen::Vector2d> transformed_point{f_x};

    transformed_point = target - transform_ * source;
  }

  Eigen::Affine2d transform_;
};
} // namespace mslam