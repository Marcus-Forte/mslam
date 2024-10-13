#pragma once

#include "Voxel.hh"
#include "map/IMap2D.hh"
#include <Eigen/Dense>

namespace mslam {
class VoxelHashMap : public IMap2D {
public:
  using Voxel = Eigen::Vector2i;

  VoxelHashMap(float voxel_size, size_t max_points_per_voxel);
  void addScan(const PointCloud2 &scan);

  Neighbor getClosestNeighbor(const Point2 &query) const;

  /**
   * @brief Get a Point Cloud Representation. Copy might be made.
   *
   * @return PointCloud2D
   */
  const PointCloud2 &getPointCloudRepresentation() const;

  void setNumAdjacentVoxelSearch(int adjacent_voxels);

private:
  float voxel_size_;
  size_t max_points_per_voxel;
  int adjacent_voxels_;

  std::unordered_map<Voxel2, PointCloud2> map_;
  mutable PointCloud2 map_rep_;
};

} // namespace mslam
