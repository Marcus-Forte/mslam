#include "map/KDTreeMap.hh"
#include "pcl/memory.h"
#include "pcl/point_cloud.h"
#include "pcl/types.h"
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
constexpr float g_voxelSize = 0.05;

namespace mslam {

KDTreeMap::KDTreeMap() { map_cloud_ = pcl::make_shared<PointCloudT>(); }

void KDTreeMap::addScan(const PointCloud3 &pointcloud) {
  for (const auto &pt : pointcloud.points) {
    map_cloud_->emplace_back(pt.x, pt.y, pt.z);
  }
  // Subsample
  pcl::VoxelGrid<Point3> voxel;
  voxel.setMinimumPointsNumberPerVoxel(1);
  voxel.setLeafSize(g_voxelSize, g_voxelSize, g_voxelSize);
  voxel.setInputCloud(map_cloud_);
  voxel.filter(*map_cloud_);

  map_rep_.points.resize(map_cloud_->size());
  std::transform(map_cloud_->begin(), map_cloud_->end(),
                 map_rep_.points.begin(),
                 [](const Point3 &pt) -> Point3 { return {pt.x, pt.y, pt.z}; });

  kdtree_.setInputCloud(map_cloud_);
}

const PointCloud3 &KDTreeMap::getPointCloudRepresentation() const {
  return map_rep_;
}

IMap::Neighbor KDTreeMap::getClosestNeighbor(const Point3 &query) const {
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  kdtree_.nearestKSearch(query, 1, index, sqr_distances);
  const auto nearest = map_cloud_->points[index[0]];
  return {{nearest.x, nearest.y, nearest.z}, sqr_distances[0]};
}
} // namespace mslam
