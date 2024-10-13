#pragma once

#include "common/Points.hh"
#include <Eigen/Dense>

namespace mslam {
using Voxel2 = Eigen::Vector2i;
inline Voxel2 PointToVoxel(const Point2 &point, float voxel_size) {
  return {static_cast<int>(std::floor(point.x() / voxel_size)),
          static_cast<int>(std::floor(point.y() / voxel_size))};
}

} // namespace mslam

template <> struct std::hash<mslam::Voxel2> {
  std::size_t operator()(const mslam::Voxel2 &voxel) const;
};