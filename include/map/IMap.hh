#pragma once

#include "common/Points.hh"
#include <vector>

namespace mslam {
class IMap {
public:
  /**
   * @brief A neighbor in the map.
   *
   * A neighbor consists of a point in the map and its associated distance
   * metric (squared distance).
   */
  using Neighbor = std::pair<Point, float>;
  /**
   * @brief Add points to the map.
   *
   * @param scan Points to be added to the map.
   * @return PointCloud The subset of points that were actually inserted.
   */
  virtual PointCloud addScan(const PointCloud &scan) = 0;

  /**
   * @brief Return the single closest neighbor to the query point.
   *
   * @param query Query point in map coordinates.
   * @return Neighbor Closest point and its distance metric.
   */
  virtual Neighbor getClosestNeighbor(const Point &query) const = 0;

  /**
   * @brief Return up to the closest N neighbors to the query point.
   *
   * Results should be ordered from nearest to farthest according to the same
   * distance metric used by getClosestNeighbor(). Implementations should
   * return an empty vector when N <= 0 or when no candidates are available.
   *
   * @param query Query point in map coordinates.
   * @param N Maximum number of neighbors to return.
   * @return std::vector<Neighbor> Up to N nearest neighbors ordered by
   * distance.
   */
  virtual std::vector<Neighbor> getClosestNNeighbors(const Point &query,
                                                     int N) const = 0;

  /**
   * @brief Get a Point Cloud Representation. Copy might be made.
   *
   * @return PointCloud
   */
  virtual const PointCloud &getPointCloudRepresentation() const = 0;

  /**
   * @brief Remove all points from the map.
   */
  virtual void clear() = 0;
};
} // namespace mslam