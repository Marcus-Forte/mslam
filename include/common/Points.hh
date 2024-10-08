#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mslam {
using Point2 = Eigen::Vector2f;
using Point3 = Eigen::Vector3f;

struct PointCloud2D {
  long timestamp;
  std::vector<Point2> points;
};

struct PointCloud3D {
  long timestamp;
  std::vector<Point3> points;
};

} // namespace mslam