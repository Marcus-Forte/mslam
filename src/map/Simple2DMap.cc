#include "map/Simple2DMap.hh"

namespace mslam {

SimpleMap2D::SimpleMap2D() : map_{} {};
void SimpleMap2D::addScan(const PointCloud2D &pointcloud) {
  map_.points.insert(map_.points.end(), pointcloud.points.begin(),
                     pointcloud.points.end());
}

const mslam::PointCloud2D &SimpleMap2D::getPointCloudRepresentation() const {
  return map_;
}
} // namespace mslam
