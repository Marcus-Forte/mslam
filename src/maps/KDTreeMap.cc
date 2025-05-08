#include "map/KDTreeMap.hh"
#include "pcl/memory.h"
#include "pcl/point_cloud.h"
#include "pcl/types.h"
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace mslam {

KDTreeMap::KDTreeMap() { map_rep_ = pcl::make_shared<PointCloudT>(); }

void KDTreeMap::addScan(const PointCloud3 &pointcloud) {
  *map_rep_ += pointcloud;

  /// \todo Subsample?

  kdtree_.setInputCloud(map_rep_);
}

const PointCloud3 &KDTreeMap::getPointCloudRepresentation() const {
  return *map_rep_;
}

IMap::Neighbor KDTreeMap::getClosestNeighbor(const Point3 &query) const {
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  kdtree_.nearestKSearch(query, 1, index, sqr_distances);
  const auto nearest = map_rep_->points[index[0]];
  return {{nearest.x, nearest.y, nearest.z}, sqr_distances[0]};
}
} // namespace mslam
