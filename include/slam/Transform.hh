#pragma once

#include <Eigen/Dense>

Eigen::Affine2d toAffine(double x, double y, double theta);
Eigen::Affine3d toAffine(double x, double y, double z, double rx, double ry,
                         double rz);