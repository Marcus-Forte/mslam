#pragma once

#include "common/Points.hh"
#include <Eigen/Dense>

namespace mslam {

using Voxel3 = Eigen::Vector3i;

inline Voxel3 PointToVoxel(const Point &point, float inverse_voxel_size) {
  return {static_cast<int>(std::floor(point.x * inverse_voxel_size)),
          static_cast<int>(std::floor(point.y * inverse_voxel_size)),
          static_cast<int>(std::floor(point.z * inverse_voxel_size))};
}

} // namespace mslam

template <> struct std::hash<mslam::Voxel3> {
  std::size_t operator()(const mslam::Voxel3 &voxel) const;
};