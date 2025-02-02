#pragma once

#include "Voxel.hh"
#include "map/IMap.hh"
#include <Eigen/Dense>

namespace mslam {
class VoxelHashMap : public IMap {
public:
  VoxelHashMap(float voxel_size, size_t max_points_per_voxel);
  void addScan(const PointCloud3 &scan);

  Neighbor getClosestNeighbor(const Point3 &query) const;
  const PointCloud3 &getPointCloudRepresentation() const;
  void setNumAdjacentVoxelSearch(int adjacent_voxels);

private:
  float voxel_size_;
  size_t max_points_per_voxel_;
  int adjacent_voxels_;

  std::unordered_map<Voxel3, PointCloud3> map_;
  mutable PointCloud3 map_rep_;
};

} // namespace mslam
