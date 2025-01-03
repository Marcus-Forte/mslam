#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mslam {
using Point2 = Eigen::Vector2d;
using Point3 = Eigen::Vector3d;

using PointCloud2 = std::vector<Point2>;
using PointCloud3 = std::vector<Point3>;

struct PointCloud2T {
  uint64_t timestamp;
  PointCloud2 points;
};

} // namespace mslam