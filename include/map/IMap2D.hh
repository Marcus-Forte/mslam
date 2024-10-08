#pragma once

#include "common/Points.hh"

namespace mslam {
class IMap2D {
public:
  /**
   * @brief Add points to the map.
   *
   * @param points
   */
  virtual void addScan(const PointCloud2D &scan) = 0;

  /**
   * @brief Get a Point Cloud Representation. Copy will be made.
   *
   * @return PointCloud2D
   */
  virtual const PointCloud2D &getPointCloudRepresentation() const = 0;
};
} // namespace mslam