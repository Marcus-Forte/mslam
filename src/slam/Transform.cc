#include "slam/Transform.hh"

#include "pcl/common//transforms.h"

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

/// \todo how to prevent conversions to double?
void transformCloud(const Eigen::Affine3d &transform,
                    msensor::PointCloud3 &cloud_out) {

  // pcl::transformPointCloud(cloud_out, cloud_out, transform.matrix());
  std::for_each(cloud_out.begin(), cloud_out.end(), [&](auto &pt) {
    const Eigen::Vector3d point_eigen{pt.x, pt.y, pt.z};
    const Eigen::Vector3d transformed_scan_pt = transform * point_eigen;
    pt.x = transformed_scan_pt.x();
    pt.y = transformed_scan_pt.y();
    pt.z = transformed_scan_pt.z();
  });
}

/// \todo how to prevent conversions to double?
void transformCloud(const Eigen::Affine2d &transform,
                    msensor::PointCloud3 &cloud_out) {

  // pcl::transformPointCloud(cloud_out, cloud_out, transform.matrix());
  std::for_each(cloud_out.begin(), cloud_out.end(), [&](auto &pt) {
    const Eigen::Vector2d point_eigen{pt.x, pt.y};
    const Eigen::Vector2d transformed_scan_pt = transform * point_eigen;
    pt.x = transformed_scan_pt.x();
    pt.y = transformed_scan_pt.y();
  });
}