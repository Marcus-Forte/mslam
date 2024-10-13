#include "map/KDTree2DMap.hh"
#include "common/Points.hh"
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

KDTree2DMap::KDTree2DMap() { map_cloud_ = pcl::make_shared<PointCloudT>(); }

void KDTree2DMap::addScan(const PointCloud2 &pointcloud) {
  for (const auto &pt : pointcloud) {
    map_cloud_->emplace_back(pt[0], pt[1], 0);
  }
  // Subsample
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setMinimumPointsNumberPerVoxel(1);
  voxel.setLeafSize(g_voxelSize, g_voxelSize, g_voxelSize);
  voxel.setInputCloud(map_cloud_);
  voxel.filter(*map_cloud_);

  map_rep_.resize(map_cloud_->size());
  std::transform(
      map_cloud_->begin(), map_cloud_->end(), map_rep_.begin(),
      [](const pcl::PointXYZ &pt) -> Point2 { return {pt.x, pt.y}; });

  kdtree_.setInputCloud(map_cloud_);
}

const mslam::PointCloud2 &KDTree2DMap::getPointCloudRepresentation() const {
  return map_rep_;
}

IMap2D::Neighbor KDTree2DMap::getClosestNeighbor(const Point2 &query) const {
  pcl::PointXYZ pt(query[0], query[1], 0);
  pcl::Indices index(1);
  std::vector<float> sqr_distances(1);
  kdtree_.nearestKSearch(pt, 1, index, sqr_distances);
  const auto nearest = map_cloud_->points[index[0]];
  return {{nearest.x, nearest.y}, sqr_distances[0]};
}
} // namespace mslam
