#include "mslam/slam/Transform.hh"

#include <algorithm>

namespace mslam {

Eigen::Affine2d toAffine(double x, double y, double theta) {
  Eigen::Affine2d transform = Eigen::Affine2d::Identity();
  transform.translate(Eigen::Vector2d{x, y});
  transform.rotate(theta);
  return transform;
}

/// \todo optimize SO3 operations
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz) {
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  Eigen::AngleAxisd roll(rx, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(rz, Eigen::Vector3d::UnitZ());
  transform.translate(Eigen::Vector3d{x, y, z});
  transform.rotate(roll * pitch * yaw);
  return transform;
}

Eigen::Affine3d toAffine(const SlamState &state) {
  return toAffine(state.position.x(), state.position.y(), state.position.z(),
                  state.rotation.x(), state.rotation.y(), state.rotation.z());
}

Eigen::Matrix3d eulerToRotation(const Eigen::Vector3d &euler) {
  return toAffine(0.0, 0.0, 0.0, euler.x(), euler.y(), euler.z()).linear();
}

Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d &rotation) {
  return {std::atan2(-rotation(1, 2), rotation(2, 2)),
          std::asin(std::clamp(rotation(0, 2), -1.0, 1.0)),
          std::atan2(-rotation(0, 1), rotation(0, 0))};
}

void setPose(SlamState &state, const Eigen::Affine3d &pose) {
  state.position = pose.translation();
  state.rotation = rotationToEuler(pose.linear());
}

SlamState toSlamState(const Eigen::Affine3d &pose) {
  SlamState state;
  setPose(state, pose);
  return state;
}

/// \todo how to prevent conversions to double?
void transformCloud(const Eigen::Affine3d &transform, PointCloud &cloud_out) {

  std::for_each(cloud_out.begin(), cloud_out.end(), [&](auto &pt) {
    const Eigen::Vector3d point_eigen{pt.x, pt.y, pt.z};
    const Eigen::Vector3d transformed_scan_pt = transform * point_eigen;
    pt.x = transformed_scan_pt.x();
    pt.y = transformed_scan_pt.y();
    pt.z = transformed_scan_pt.z();
  });
}

/// \todo how to prevent conversions to double?
void transformCloud(const Eigen::Affine2d &transform, PointCloud &cloud_out) {

  std::for_each(cloud_out.begin(), cloud_out.end(), [&](auto &pt) {
    const Eigen::Vector2d point_eigen{pt.x, pt.y};
    const Eigen::Vector2d transformed_scan_pt = transform * point_eigen;
    pt.x = transformed_scan_pt.x();
    pt.y = transformed_scan_pt.y();
  });
}

} // namespace mslam
