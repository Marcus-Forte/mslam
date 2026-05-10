#pragma once

#include "common/Points.hh"
#include <vector>

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

  /**
   * @brief Return the single closest neighbor to the query point.
   *
   * @param query Query point in map coordinates.
   * @return Neighbor Closest point and its distance metric.
   */
  virtual Neighbor getClosestNeighbor(const Point3 &query) const = 0;

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
  virtual std::vector<Neighbor> getClosestNNeighbors(const Point3 &query,
                                                     int N) const = 0;

  /**
   * @brief Return all neighbors within a search radius of the query point.
   *
   * Results should be ordered from nearest to farthest according to the same
   * distance metric used by getClosestNeighbor(). Implementations should
   * return an empty vector when radius <= 0 or when no candidates are found.
   *
   * @param query Query point in map coordinates.
   * @param radius Search radius in map units.
   * @return std::vector<Neighbor> All neighbors within the radius, ordered by
   * distance.
   */
  virtual std::vector<Neighbor>
  getClosestNeighborsRadius(const Point3 &query, float radius) const = 0;

  /**
   * @brief Get a Point Cloud Representation. Copy might be made.
   *
   * @return PointCloud3
   */
  virtual const PointCloud3 &getPointCloudRepresentation() const = 0;
};
} // namespace mslam