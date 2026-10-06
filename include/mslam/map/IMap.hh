#pragma once

#include "mslam/common/Points.hh"
#include <vector>

namespace mslam {
class IMap {
public:
  virtual ~IMap() = default;
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
   * @brief Bound the map around the current pose.
   *
   * Keeps the map size (and therefore nearest-neighbour lookup cost)
   * independent of how long the session has been running. Implementations
   * may ignore values <= 0 to disable a bound.
   *
   * @param center Current pose position in map coordinates.
   * @param max_range Drop entries farther than this from `center` (m,
   *        <= 0 disables).
   * @param max_voxels Hard upper bound on the number of stored voxels,
   *        evicting those farthest from `center` (<= 0 disables).
   */
  virtual void prune(const Point &center, float max_range, size_t max_voxels) {
    (void)center;
    (void)max_range;
    (void)max_voxels;
  }

  /**
   * @brief Remove all points from the map.
   */
  virtual void clear() = 0;
};
} // namespace mslam