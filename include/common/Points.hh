#pragma once

#include "lidar/ILidar.hh"
#include <Eigen/Dense>

namespace mslam {

using Point3 = msensor::Point3;
using PointCloud3 = msensor::PointCloud3;

using VectorPoint3f = std::vector<Eigen::Vector3f>;
using VectorPoint2f = std::vector<Eigen::Vector2f>;

using VectorPoint3d = std::vector<Eigen::Vector3d>;
using VectorPoint2d = std::vector<Eigen::Vector2d>;

} // namespace mslam