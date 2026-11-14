#pragma once

#include "mslam/common/Points.hh"
#include "mslam/map/IMap.hh"
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <tsl/robin_map.h>

namespace mslam {

/// Estimates surface normals for map points using PCA on KNN neighborhoods.
///
/// Results are cached per map point for the lifetime of one registration
/// (between two calls to reset()), so the repeated registration iterations of
/// one scan do not recompute them. The cache is never carried across scans:
/// the map changes in between. The estimator is meant to be a long-lived
/// member of a registration; reset() keeps the allocated capacity.
class NormalEstimator {
public:
  /// Largest supported neighborhood size.
  static constexpr int kMaxNeighbors = 16;

  explicit NormalEstimator(int num_neighbors = 5);

  /// Start a new registration against `map`: drops all cached normals (but not
  /// their storage). `map` must outlive the following estimate() calls.
  void reset(const IMap &map);

  /// Returns the estimated surface normal for the given map point, or
  /// std::nullopt if it has fewer than 3 neighbors or they are degenerate.
  std::optional<Eigen::Vector3d> estimate(const Point &query);

  /// Normal of the best-fit plane through `neighbors`. Exposed for testing.
  static std::optional<Eigen::Vector3d>
  fromNeighbors(std::span<const IMap::Neighbor> neighbors);

  /// Number of distinct map points currently cached.
  std::size_t cachedCount() const { return cache_.size(); }

private:
  // Keyed on the exact bit pattern of the coordinates, so hashing and equality
  // agree (+0.0 and -0.0 are different keys, which only costs a cache miss).
  struct MapPointKey {
    std::uint32_t x, y, z;
    bool operator==(const MapPointKey &) const = default;
  };

  struct MapPointKeyHash {
    std::size_t operator()(const MapPointKey &key) const noexcept {
      std::uint64_t h = (static_cast<std::uint64_t>(key.x) << 32 | key.y) *
                        0x9E3779B97F4A7C15ULL;
      h ^= static_cast<std::uint64_t>(key.z) * 0xC2B2AE3D27D4EB4FULL;
      h ^= h >> 32;
      h *= 0xD6E8FEB86659FD93ULL;
      h ^= h >> 32;
      return static_cast<std::size_t>(h);
    }
  };

  const IMap *map_ = nullptr;
  int num_neighbors_;
  tsl::robin_map<MapPointKey, std::optional<Eigen::Vector3d>, MapPointKeyHash>
      cache_;
};

} // namespace mslam
