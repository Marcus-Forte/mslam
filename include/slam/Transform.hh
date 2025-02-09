#pragma once

#include "lidar/ILidar.hh"
#include <Eigen/Dense>

Eigen::Affine2d toAffine(double x, double y, double theta);
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz);

void transformCloud(const Eigen::Affine3d &transform,
                    msensor::PointCloud3 &cloud_out);