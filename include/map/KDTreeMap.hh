#pragma once

#include "map/IMap.hh"
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

namespace mslam {
class KDTreeMap : public IMap {
public:
  KDTreeMap();
  void addScan(const PointCloud3 &points) override;

  const PointCloud3 &getPointCloudRepresentation() const override;

  Neighbor getClosestNeighbor(const Point3 &query) const override;

private:
  pcl::search::KdTree<pcl::PointXYZ> kdtree_;
  PointCloud3::Ptr map_rep_;
};
} // namespace mslam