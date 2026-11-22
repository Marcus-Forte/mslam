#pragma once

#include "mslam/common/Points.hh"

#include <cstddef>
#include <cstdint>

namespace mslam {

class SyclVoxelHashMap;

/// What the correspondence kernel found for one source point. Written by that
/// point's work-item only.
struct DeviceCorrespondence {
  float map_point[3];   ///< Nearest map point (valid when matched).
  std::uint32_t slot;   ///< Its pool slot in the map (valid when matched).
  std::uint8_t matched; ///< Nearest point within the correspondence gate.
  std::uint8_t computed;   ///< Its normal was computed in this launch.
  std::uint8_t has_normal; ///< `normal` is valid.
  double normal[3];
};

/// Total kernel launches of all DeviceCorrespondenceSearch objects (bench_slam).
std::uint64_t deviceCorrespondenceLaunches();

/**
 * @brief The correspondence search and normal estimation of a registration
 * round, in one SYCL kernel over the source points.
 *
 * For every source point: the nearest map point (device::closestNeighbor() on
 * the map's mirror, the same search as VoxelHashMap::getClosestNeighbor()), the
 * correspondence gate, and for a match the map point's surface normal, from the
 * map's normal cache or, on a miss, from its `num_neighbors` nearest map points
 * (normalFromPoints(), as NormalEstimator does). Each point is handled by one
 * work-item that writes only its own result, and the `width` work-groups
 * claim runs of 32 points dynamically (balancing the varying cost of a point),
 * so the results do not depend on the width or the scheduling. The caller
 * stores the computed normals into the cache and assembles the constraints in
 * point order (IcpWorkspace).
 */
class DeviceCorrespondenceSearch {
public:
  explicit DeviceCorrespondenceSearch(int num_neighbors = 5);
  ~DeviceCorrespondenceSearch();

  DeviceCorrespondenceSearch(const DeviceCorrespondenceSearch &) = delete;
  DeviceCorrespondenceSearch &
  operator=(const DeviceCorrespondenceSearch &) = delete;

  /// Runs the kernel for @p source (map coordinates) on @p width work-groups
  /// and waits; returns source.size() results, valid until the next run().
  const DeviceCorrespondence *run(const SyclVoxelHashMap &map,
                                  const PointCloud &source,
                                  float max_correspondence_distance,
                                  std::size_t width);

private:
  void reserve(const SyclVoxelHashMap &map, std::size_t points);

  int num_neighbors_;
  const SyclVoxelHashMap *allocated_for_ = nullptr; ///< Owner of the buffers' queue.
  DeviceCorrespondence *results_ = nullptr;
  Point *source_copy_ = nullptr; ///< For devices that cannot read host memory.
  std::size_t *next_chunk_ = nullptr; ///< Claim counter of the work-groups.
  std::size_t capacity_ = 0;
};

} // namespace mslam
