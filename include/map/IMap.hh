#pragma once

#include "common/Points.hh"

namespace mslam {
class IMap {
public:
  using Neighbor = std::pair<Point3, float>;
  /**
   * @brief Add points to the map.
   *
   * @param points
   */
  virtual void addScan(const PointCloud3 &scan) = 0;

  virtual Neighbor getClosestNeighbor(const Point3 &query) const = 0;

  /**
   * @brief Get a Point Cloud Representation. Copy might be made.
   *
   * @return PointCloud2D
   */
  virtual const PointCloud3 &getPointCloudRepresentation() const = 0;
};
} // namespace mslam