#include "map/VoxelHashMap.hh"

#include <algorithm>

namespace mslam {

static std::vector<Voxel3> buildVoxelShifts(int adjacent_voxels) {
  std::vector<Voxel3> shifts;
  for (int dx = -adjacent_voxels; dx <= adjacent_voxels; ++dx)
    for (int dy = -adjacent_voxels; dy <= adjacent_voxels; ++dy)
      for (int dz = -adjacent_voxels; dz <= adjacent_voxels; ++dz)
        shifts.emplace_back(dx, dy, dz);
  return shifts;
}

VoxelHashMap::VoxelHashMap(float voxel_size, size_t max_points_per_voxel)
    : voxel_size_(voxel_size), max_points_per_voxel_(max_points_per_voxel),
      adjacent_voxels_(1), voxel_shifts_(buildVoxelShifts(1)) {}

void VoxelHashMap::addScan(const PointCloud3 &scan) {

  for (const auto &point : scan.points) {

    const auto voxel = PointToVoxel({point.x, point.y, point.z}, voxel_size_);

    auto search = map_.find(voxel);
    if (search != map_.end()) {
      auto &bucket = search->second;
      if (bucket.points.size() != max_points_per_voxel_) {
        bucket.points.emplace_back(point);
      }
    } else { // new bucket
      PointCloud3 new_bucket;
      new_bucket.points.reserve(max_points_per_voxel_);
      new_bucket.points.emplace_back(point);
      map_.insert({voxel, std::move(new_bucket)});
    }
  }
}

IMap::Neighbor VoxelHashMap::getClosestNeighbor(const Point3 &query) const {
  const auto voxel = PointToVoxel(query, voxel_size_);
  const Eigen::Vector3f query_eigen{query.x, query.y, query.z};

  IMap::Neighbor best_neighbor{{0, 0, 0}, std::numeric_limits<float>::max()};

  for (const auto &voxel_shift : voxel_shifts_) {
    const auto query_voxel = voxel + voxel_shift;
    const auto search = map_.find(query_voxel);
    if (search == map_.end()) {
      continue;
    }

    const auto &bucket_points = search->second;
    for (const auto &point : bucket_points.points) {
      const float squared_distance =
          (point.getVector3fMap() - query_eigen).squaredNorm();
      if (squared_distance < best_neighbor.second) {
        best_neighbor = {Point3{point.x, point.y, point.z}, squared_distance};
      }
    }
  }

  return best_neighbor;
}

std::vector<IMap::Neighbor>
VoxelHashMap::getClosestNNeighbors(const Point3 &query, int N) const {
  std::vector<IMap::Neighbor> neighbors;
  if (N <= 0) {
    return neighbors;
  }

  const auto &voxel = PointToVoxel(query, voxel_size_);

  const Eigen::Vector3f query_eigen{query.x, query.y, query.z};

  for (const auto &voxel_shift : voxel_shifts_) {
    const auto query_voxel = voxel + voxel_shift;
    auto search = map_.find(query_voxel);
    if (search != map_.end()) {
      const auto &bucket_points = search->second;
      for (const auto &point : bucket_points.points) {
        const float squared_distance =
            (point.getVector3fMap() - query_eigen).squaredNorm();
        neighbors.emplace_back(Point3{point.x, point.y, point.z},
                               squared_distance);
      }
    }
  }

  std::sort(
      neighbors.begin(), neighbors.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });

  if (neighbors.size() > static_cast<std::size_t>(N)) {
    neighbors.resize(static_cast<std::size_t>(N));
  }

  return neighbors;
}

/**
 * @brief Get a Point Cloud Representation. Copy might be made.
 *
 * @return PointCloud2D
 */
const PointCloud3 &VoxelHashMap::getPointCloudRepresentation() const {
  map_rep_.points.clear();
  map_rep_.points.reserve(map_.size() *
                          static_cast<size_t>(max_points_per_voxel_));
  std::for_each(map_.cbegin(), map_.cend(), [&](const auto &map_element) {
    const auto &voxel_buckets = map_element.second;
    map_rep_.points.insert(map_rep_.points.end(), voxel_buckets.points.cbegin(),
                           voxel_buckets.points.cend());
  });
  // map_rep_.shrink_to_fit();
  return map_rep_;
}

const float VoxelHashMap::getResolution() const { return voxel_size_; }

/**
 * @brief Set number of adjacent voxels from the query point to search for
 * bucket\s. This number exponentially decreases performance, albeit to as much
 * for low numbers.
 * @param adjacent_voxels
 */
void VoxelHashMap::setNumAdjacentVoxelSearch(int adjacent_voxels) {
  adjacent_voxels_ = adjacent_voxels;
  voxel_shifts_ = buildVoxelShifts(adjacent_voxels);
}

} // namespace mslam
