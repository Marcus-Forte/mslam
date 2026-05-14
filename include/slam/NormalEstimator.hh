#pragma once

#include "common/Points.hh"
#include "map/IMap.hh"
#include <Eigen/Dense>
#include <optional>
#include <unordered_map>

namespace mslam {

/// Estimates surface normals for map points using PCA on KNN neighborhoods.
/// Caches results to avoid redundant computation across registration
/// iterations.
class NormalEstimator {
public:
  explicit NormalEstimator(const IMap &map, int num_neighbors = 5);

  /// Returns the estimated surface normal for the given map point.
  /// Caches results to avoid redundant KNN lookups across iterations.
  std::optional<Eigen::Vector3d> estimate(const Point &query);

private:
  struct MapPointKey {
    float x, y, z;
    bool operator==(const MapPointKey &) const = default;
  };

  struct MapPointKeyHash {
    std::size_t operator()(const MapPointKey &key) const;
  };

  static std::optional<Eigen::Vector3d>
  fromNeighbors(const std::vector<IMap::Neighbor> &neighbors);

  const IMap &map_;
  int num_neighbors_;
  std::unordered_map<MapPointKey, std::optional<Eigen::Vector3d>,
                     MapPointKeyHash>
      cache_;
};

} // namespace mslam
