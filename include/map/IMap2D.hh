#pragma once

#include "common/Points.hh"

namespace mslam {
class IMap2D {
public:
  using Neighbor = std::pair<Point2, double>;
  /**
   * @brief Add points to the map.
   *
   * @param points
   */
  virtual void addScan(const PointCloud2 &scan) = 0;

  virtual Neighbor getClosestNeighbor(const Point2 &query) const = 0;

  /**
   * @brief Get a Point Cloud Representation. Copy might be made.
   *
   * @return PointCloud2D
   */
  virtual const PointCloud2 &getPointCloudRepresentation() const = 0;
};
} // namespace mslam