#include "map/OctreeMap.hh"
#include "common/Points.hh"

namespace mslam {

OctreeMap::OctreeMap(float voxel_size)
    : voxel_size_(voxel_size), octree_(voxel_size) {
  map_rep_ = pcl::make_shared<PointCloud>();
  map_centers_rep = pcl::make_shared<PointCloud>();
}

PointCloud OctreeMap::addScan(const PointCloud &scan) {
  *map_rep_ += scan;
  /// \todo octree supports adding points at a time, but how to get the right
  /// indices from neighbor search?
  octree_.setInputCloud(map_rep_);
  octree_.addPointsFromInputCloud();
  return scan;
}

/// \todo If query is 3x beyond points, how to indicate to caller?
IMap::Neighbor OctreeMap::getClosestNeighbor(const Point &query) const {
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  octree_.nearestKSearch(query, 1, index, sqr_distances);
  const auto nearest = map_rep_->points[index[0]];
  return {{nearest.x, nearest.y, nearest.z}, sqr_distances[0]};
}

std::vector<IMap::Neighbor> OctreeMap::getClosestNNeighbors(const Point &query,
                                                            int N) const {
  if (N <= 0) {
    return {};
  }

  return {getClosestNeighbor(query)};
}

const PointCloud &OctreeMap::getPointCloudRepresentation() const {

  octree_.getOccupiedVoxelCenters(map_centers_rep->points);
  return *map_centers_rep;
}

void OctreeMap::clear() {
  octree_.deleteTree();
  map_rep_->clear();
  map_centers_rep->clear();
}

} // namespace mslam