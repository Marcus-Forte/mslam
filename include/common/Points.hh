#pragma once

#include "msensor/interface/ILidar.hh"
#include <Eigen/Dense>

namespace mslam {

using PointCloud = msensor::PointCloud3I;
using Point = PointCloud::PointType;
using Scan = msensor::Scan3DI;

using VectorPoint3d = std::vector<Eigen::Vector3d>;
using VectorPoint2d = std::vector<Eigen::Vector2d>;

} // namespace mslam