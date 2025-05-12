#include "map/OctreeMap.hh"

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace mslam {

OctreeMap::OctreeMap(float voxel_size)
    : voxel_size_(voxel_size), octree_(voxel_size) {
  map_rep_ = pcl::make_shared<PointCloudT>();
  map_centers_rep = pcl::make_shared<PointCloudT>();
}

void OctreeMap::addScan(const PointCloud3 &scan) {
  *map_rep_ += scan;
  /// \todo octree supports adding points at a time, but how to get the right
  /// indices from neighbor search?
  octree_.setInputCloud(map_rep_);
  octree_.addPointsFromInputCloud();
}

/// \todo If query is 3x beyond points, how to indicate to caller?
IMap::Neighbor OctreeMap::getClosestNeighbor(const Point3 &query) const {
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  octree_.nearestKSearch(query, 1, index, sqr_distances);
  const auto nearest = map_rep_->points[index[0]];
  return {{nearest.x, nearest.y, nearest.z}, sqr_distances[0]};
}
const PointCloud3 &OctreeMap::getPointCloudRepresentation() const {

  octree_.getOccupiedVoxelCenters(map_centers_rep->points);
  return *map_centers_rep;
}

} // namespace mslam