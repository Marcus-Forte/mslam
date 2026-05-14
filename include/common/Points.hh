#pragma once

#include "msensor/interface/ILidar.hh"
#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace mslam {

using PointCloud = msensor::PointCloud3I;
using Point = PointCloud::PointType;
using Scan = msensor::Scan3DI;

// Correspondence layout: [source scan point, target map point].
using Correspondence = std::pair<Point, Point>;
using Correspondences = std::vector<Correspondence>;

using VectorPoint3d = std::vector<Eigen::Vector3d>;

} // namespace mslam