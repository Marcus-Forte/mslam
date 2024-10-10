#include "Transform.hh"

Eigen::Affine2d toAffine(double x, double y, double theta) {
  Eigen::Affine2d transform;
  transform.setIdentity();
  transform.rotate(theta);
  transform.translate(Eigen::Vector2d{x, y});
  return transform;
}