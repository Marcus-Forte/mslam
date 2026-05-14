#pragma once

#include "map/IMap.hh"
#include "map/VoxelHashMap.hh"
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

namespace mslam {
class KDTreeMap : public IMap {
public:
  explicit KDTreeMap(float resolution);

  PointCloud addScan(const PointCloud &points) override;
  const PointCloud &getPointCloudRepresentation() const override;
  void clear() override;
  Neighbor getClosestNeighbor(const Point &query) const override;
  std::vector<Neighbor> getClosestNNeighbors(const Point &query,
                                             int N) const override;

private:
  VoxelHashMap voxel_map_;
  pcl::search::KdTree<Point> kdtree_;
  PointCloud::Ptr map_rep_;
};
} // namespace mslam