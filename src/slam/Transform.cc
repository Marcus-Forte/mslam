#include "slam/Transform.hh"

Eigen::Affine2d toAffine(double x, double y, double theta) {
  Eigen::Affine2d transform = Eigen::Affine2d::Identity();
  transform.rotate(theta);
  transform.translate(Eigen::Vector2d{x, y});
  return transform;
}

/// \todo optimize SO3 operations
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz) {
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  Eigen::AngleAxisd roll(rx, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(rz, Eigen::Vector3d::UnitZ());
  transform.rotate(roll * pitch * yaw);
  transform.translate(Eigen::Vector3d{x, y, z});
  return transform;
}