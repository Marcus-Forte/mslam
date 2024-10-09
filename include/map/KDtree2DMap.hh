#pragma once

#include "common/Points.hh"
#include "map/IMap2D.hh"
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

namespace mslam {
class KDTree2DMap : public IMap2D {
public:
  KDTree2DMap();
  void addScan(const PointCloud2D &points) override;

  const PointCloud2D &getPointCloudRepresentation() const override;

  Neighbor getClosestNeighbor(const Point2 &query) const override;

private:
  pcl::search::KdTree<pcl::PointXYZ> kdtree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  PointCloud2D map_;
};
} // namespace mslam