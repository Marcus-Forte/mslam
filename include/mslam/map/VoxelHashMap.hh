#pragma once

#include "mslam/map/IMap.hh"
#include "mslam/map/Voxel.hh"
#include <Eigen/Dense>
#include <cstdint>
#include <tsl/robin_map.h>

namespace mslam {

/**
 * @brief Voxel hash map holding up to `max_points_per_voxel` points per voxel.
 *
 * The points of all voxels live in one contiguous pool; each voxel owns a
 * fixed-size chunk of `max_points_per_voxel` entries, addressed through the
 * hash map. A lookup therefore costs one hash probe plus a read of contiguous
 * memory, without a separate heap block per voxel. Chunks of pruned voxels are
 * recycled.
 */
class VoxelHashMap : public IMap {
public:
  /// Largest supported neighbourhood radius, in voxels (see
  /// setNumAdjacentVoxelSearch()).
  static constexpr int kMaxAdjacentVoxels = 8;

  VoxelHashMap(float voxel_size, size_t max_points_per_voxel);
  PointCloud addScan(const PointCloud &scan) override;

  /// Exhaustive search of the voxel neighbourhood, but neighbouring voxels
  /// that cannot contain a closer point than the best found so far are not
  /// looked up. Returns {0,0,0} at the maximum float distance when the
  /// neighbourhood is empty.
  /// \todo If query is 3x beyond points, how to indicate to caller?
  Neighbor getClosestNeighbor(const Point &query) const override;
  std::size_t findClosestNeighbors(const Point &query,
                                   std::span<Neighbor> out) const override;
  const PointCloud &getPointCloudRepresentation() const override;
  void prune(const Point &center, float max_range, size_t max_voxels) override;
  void clear() override;

  /// Search radius around the query voxel, in voxels (1 = 27 voxels).
  /// \throws std::invalid_argument if outside [0, kMaxAdjacentVoxels].
  void setNumAdjacentVoxelSearch(int adjacent_voxels);

  /// Edge length of a voxel, in metres.
  float voxelSize() const { return voxel_size_; }
  /// Current search radius around the query voxel, in voxels (see
  /// setNumAdjacentVoxelSearch()).
  int numAdjacentVoxelSearch() const { return adjacent_voxels_; }

  /// Number of occupied voxels (used for diagnostics / tests).
  size_t size() const { return map_.size(); }

private:
  struct Bucket {
    std::uint32_t offset; ///< First point of the chunk in `pool_`.
    std::uint32_t count;  ///< Points stored, <= max_points_per_voxel_.
  };

  /// Squared lower bounds, per axis, between the query and each voxel of the
  /// neighbourhood along that axis (index = shift + adjacent_voxels_).
  struct AxisGaps {
    float gap[2 * kMaxAdjacentVoxels + 1][3];
  };
  AxisGaps computeAxisGaps(const Point &query, const Voxel3 &voxel) const;

  std::uint32_t allocateChunk();
  void releaseChunk(std::uint32_t offset) { free_chunks_.push_back(offset); }

  float voxel_size_;
  float inverse_voxel_size_;
  size_t max_points_per_voxel_;
  int adjacent_voxels_;
  std::vector<Voxel3> voxel_shifts_;
  size_t home_shift_index_ = 0;

  // tsl::robin_map is faster than std::unordered_map.
  tsl::robin_map<Voxel3, Bucket> map_;
  std::vector<Point> pool_;
  std::vector<std::uint32_t> free_chunks_;
  mutable PointCloud map_rep_;
  mutable bool map_rep_dirty_ = true;
  // Scratch buffer reused by prune() to avoid allocation while evicting.
  std::vector<float> prune_distances_;
};

} // namespace mslam
