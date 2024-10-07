#pragma once

#include <Eigen/Dense>
#include <vector>

using Point2 = Eigen::Vector2f;

struct PointCloud2D {
  long long timestamp;
  std::vector<Point2> points;
};