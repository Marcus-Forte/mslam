#pragma once

#include "common/Points.hh"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Affine2d toAffine(double x, double y, double theta);
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz);

void transformCloud(const Eigen::Affine3d &transform,
                    mslam::PointCloud &cloud_out);

void transformCloud(const Eigen::Affine2d &transform,
                    mslam::PointCloud &cloud_out);
