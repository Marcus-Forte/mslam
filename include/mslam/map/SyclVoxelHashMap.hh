#pragma once

#include "mslam/map/DeviceVoxelSearch.hh"
#include "mslam/map/VoxelHashMap.hh"

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace moptim {
class ExecutionContext;
}

namespace mslam {

/// The surface-normal cache of a SyclVoxelHashMap, one entry per pool slot (a
/// map point), as kernels see it. `state` is kUnknown (not computed since the
/// point's neighbourhood last changed), kNormal or kNoNormal (fewer than 3
/// neighbours, or degenerate).
struct DeviceNormalCache {
  static constexpr std::uint8_t kUnknown = 0;
  static constexpr std::uint8_t kNormal = 1;
  static constexpr std::uint8_t kNoNormal = 2;
  const std::uint8_t *state = nullptr;
  const double *normals = nullptr; ///< 3 per slot.
};

/**
 * @brief VoxelHashMap with a mirror in SYCL USM that kernels search.
 *
 * The host map stays the authority: every IMap query and the point-cloud
 * representation are the VoxelHashMap's. addScan() inserts into it and then
 * appends the same added points, in the same order, to the mirror's voxels, so
 * each voxel holds the same points in the same order and a search of view()
 * (device::closestNeighbor() / closestNeighbors()) returns what the host query
 * returns. prune() and clear() rebuild the mirror.
 *
 * It also caches the surface normal of each map point across scans, which the
 * registrations' kernels read and fill (normalCache(), storeNormal()). A map
 * point's normal depends only on the voxels within the search radius of its own
 * voxel, so when a voxel gains points the cached normals of the points in all
 * voxels within that radius of it are invalidated: what the cache holds is what
 * a fresh estimate on the current map gives.
 *
 * Not thread-safe; no kernel may read the mirror while it is modified (the
 * registrations wait for their kernels before returning).
 */
class SyclVoxelHashMap : public VoxelHashMap {
public:
  /// @param context Where the mirror is allocated (moptim's default context if
  /// null); kernels searching it run on its queue.
  SyclVoxelHashMap(float voxel_size, size_t max_points_per_voxel,
                   std::shared_ptr<moptim::ExecutionContext> context = nullptr);
  ~SyclVoxelHashMap() override;

  SyclVoxelHashMap(const SyclVoxelHashMap &) = delete;
  SyclVoxelHashMap &operator=(const SyclVoxelHashMap &) = delete;

  PointCloud addScan(const PointCloud &scan) override;
  void prune(const Point &center, float max_range, size_t max_voxels) override;
  void clear() override;

  const SyclVoxelHashMap *deviceMap() const override { return this; }

  /// The mirror as kernels search it (valid until the map changes).
  device::MapView view() const;

  DeviceNormalCache normalCache() const {
    return {normal_state_, normal_values_};
  }

  /// Stores the normal of pool slot @p slot (nullptr: the point has none).
  void storeNormal(std::uint32_t slot, const double *normal) const;

  /// Number of pool slots whose normal is cached (for tests and diagnostics).
  std::size_t cachedNormals() const;

  const std::shared_ptr<moptim::ExecutionContext> &context() const {
    return context_;
  }

private:
  /// Appends @p point to the mirror's voxel @p key; returns false if it is full.
  bool mirrorInsert(const device::VoxelKey &key, const Point &point);
  /// The table slot of @p key, inserted if absent.
  std::uint32_t findOrInsertSlot(const device::VoxelKey &key);
  void growTable(std::uint32_t capacity);
  void growPool(std::size_t slots);
  void invalidateAround(const std::vector<device::VoxelKey> &changed);
  void rebuild();
  void updateShifts() const;

  std::shared_ptr<moptim::ExecutionContext> context_;

  // The mirror (USM shared). Raw arrays, reallocated on growth.
  device::VoxelKey *keys_ = nullptr;
  device::Bucket *buckets_ = nullptr;
  std::uint32_t capacity_ = 0; ///< Table slots (a power of two).
  std::uint32_t occupied_ = 0;
  device::PointXYZI *pool_mirror_ = nullptr;
  std::size_t pool_capacity_ = 0; ///< Slots allocated.
  std::size_t pool_used_ = 0;     ///< Slots handed out to voxels.

  // The normal cache, parallel to the pool.
  std::uint8_t *normal_state_ = nullptr;
  double *normal_values_ = nullptr;

  // Shifts of the current search radius, in VoxelHashMap order.
  mutable device::VoxelKey *shifts_ = nullptr;
  mutable int shifts_radius_ = -1;
  mutable std::uint32_t num_shifts_ = 0;

  std::vector<device::VoxelKey> changed_; ///< Scratch of addScan().
  struct Probe {
    device::VoxelKey key;
    std::uint32_t home;
  };
  std::vector<Probe> probes_; ///< Scratch of invalidateAround().
};

} // namespace mslam
