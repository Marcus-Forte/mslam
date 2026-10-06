#pragma once

#include "mslam/map/IMap.hh"
#include "mslam/map/Voxel.hh"
#include <Eigen/Dense>
#include <tsl/robin_map.h>

namespace mslam {

class VoxelHashMap : public IMap {
public:
  VoxelHashMap(float voxel_size, size_t max_points_per_voxel);
  PointCloud addScan(const PointCloud &scan) override;

  /// \todo If query is 3x beyond points, how to indicate to caller?
  Neighbor getClosestNeighbor(const Point &query) const override;
  std::vector<Neighbor> getClosestNNeighbors(const Point &query,
                                             int N) const override;
  const PointCloud &getPointCloudRepresentation() const override;
  void prune(const Point &center, float max_range, size_t max_voxels) override;
  void clear() override;
  void setNumAdjacentVoxelSearch(int adjacent_voxels);

  /// Number of occupied voxels (used for diagnostics / tests).
  size_t size() const { return map_.size(); }

private:
  float voxel_size_;
  float inverse_voxel_size_;
  size_t max_points_per_voxel_;
  int adjacent_voxels_;
  std::vector<Voxel3> voxel_shifts_;

  // tsl::robin_map is faster than std::unordered_map.
  tsl::robin_map<Voxel3, PointCloud> map_;
  mutable PointCloud map_rep_;
  mutable bool map_rep_dirty_ = true;
  // Scratch buffer reused by prune() to avoid allocation while evicting.
  std::vector<float> prune_distances_;
};

} // namespace mslam
