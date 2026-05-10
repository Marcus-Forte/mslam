#include "map/KDTreeMap.hh"
#include "pcl/memory.h"
#include "pcl/point_cloud.h"
#include "pcl/types.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace mslam {

KDTreeMap::KDTreeMap() { map_rep_ = pcl::make_shared<PointCloudT>(); }

void KDTreeMap::addScan(const PointCloud3 &pointcloud) {
  *map_rep_ += pointcloud;

  kdtree_.setInputCloud(map_rep_);
}

const PointCloud3 &KDTreeMap::getPointCloudRepresentation() const {
  return *map_rep_;
}

const float KDTreeMap::getResolution() const { return 0.0F; }

IMap::Neighbor KDTreeMap::getClosestNeighbor(const Point3 &query) const {
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  kdtree_.nearestKSearch(query, 1, index, sqr_distances);
  const auto nearest = map_rep_->points[index[0]];
  return {{nearest.x, nearest.y, nearest.z}, sqr_distances[0]};
}

std::vector<IMap::Neighbor> KDTreeMap::getClosestNNeighbors(const Point3 &query,
                                                            int N) const {
  std::vector<IMap::Neighbor> neighbors;
  if (N <= 0 || map_rep_->empty()) {
    return neighbors;
  }

  pcl::Indices indices(static_cast<std::size_t>(N));
  std::vector<float> sqr_distances(static_cast<std::size_t>(N));
  const int found = kdtree_.nearestKSearch(query, N, indices, sqr_distances);
  neighbors.reserve(static_cast<std::size_t>(found));

  for (int i = 0; i < found; ++i) {
    const auto &nearest =
        map_rep_->points[indices[static_cast<std::size_t>(i)]];
    neighbors.emplace_back(Point3{nearest.x, nearest.y, nearest.z},
                           sqr_distances[static_cast<std::size_t>(i)]);
  }

  return neighbors;
}
} // namespace mslam
