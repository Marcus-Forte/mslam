#pragma once

#include "IMap.hh"
#include <pcl/octree/octree_search.h>

namespace mslam {
class OctreeMap : public IMap {
public:
  OctreeMap(float voxel_size);
  void addScan(const PointCloud3 &scan);

  /// \todo If query is 3x beyond points, how to indicate to caller?
  Neighbor getClosestNeighbor(const Point3 &query) const;
  const PointCloud3 &getPointCloudRepresentation() const;
  void setNumAdjacentVoxelSearch(int adjacent_voxels);

private:
  mutable pcl::octree::OctreePointCloudSearch<Point3> octree_;
  float voxel_size_;
  PointCloud3::Ptr map_rep_;
  PointCloud3::Ptr map_centers_rep;
};

} // namespace mslam