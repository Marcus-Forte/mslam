#include "map/KDTreeMap.hh"
#include "common/Points.hh"
#include "pcl/memory.h"
#include "pcl/types.h"
#include <pcl/kdtree/kdtree.h>
#include <vector>

namespace mslam {

KDTreeMap::KDTreeMap(float resolution)
    : voxel_map_(resolution, 1), map_rep_(pcl::make_shared<PointCloud>()) {}

PointCloud KDTreeMap::addScan(const PointCloud &pointcloud) {
  auto added = voxel_map_.addScan(pointcloud);

  *map_rep_ += added;
  kdtree_.setInputCloud(map_rep_);
  return added;
}

const PointCloud &KDTreeMap::getPointCloudRepresentation() const {
  return *map_rep_;
}

IMap::Neighbor KDTreeMap::getClosestNeighbor(const Point &query) const {
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  kdtree_.nearestKSearch(query, 1, index, sqr_distances);
  const auto nearest = map_rep_->points[index[0]];
  return {{nearest.x, nearest.y, nearest.z}, sqr_distances[0]};
}

std::vector<IMap::Neighbor> KDTreeMap::getClosestNNeighbors(const Point &query,
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
    neighbors.emplace_back(Point{nearest.x, nearest.y, nearest.z},
                           sqr_distances[static_cast<std::size_t>(i)]);
  }

  return neighbors;
}
void KDTreeMap::clear() {
  voxel_map_.clear();
  map_rep_->clear();
}
} // namespace mslam
