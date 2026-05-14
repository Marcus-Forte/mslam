#pragma once

#include "IMap.hh"
#include <pcl/octree/octree_search.h>

namespace mslam {
class OctreeMap : public IMap {
public:
  OctreeMap(float voxel_size);

  PointCloud addScan(const PointCloud &scan) override;
  Neighbor getClosestNeighbor(const Point &query) const override;
  std::vector<Neighbor> getClosestNNeighbors(const Point &query,
                                             int N) const override;
  const PointCloud &getPointCloudRepresentation() const override;
  void clear() override;

private:
  mutable pcl::octree::OctreePointCloudSearch<Point> octree_;
  float voxel_size_;
  PointCloud::Ptr map_rep_;
  PointCloud::Ptr map_centers_rep;
};

} // namespace mslam