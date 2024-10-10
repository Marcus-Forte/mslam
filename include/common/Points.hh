#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mslam {
using Point2 = Eigen::Vector2d;
using Point3 = Eigen::Vector3d;

struct PointCloud2D {
  long timestamp;
  std::vector<Point2> points;
};

struct PointCloud3D {
  long timestamp;
  std::vector<Point3> points;
};

} // namespace mslam