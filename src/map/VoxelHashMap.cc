#include "map/VoxelHashMap.hh"

#include <algorithm>

namespace {

inline float squaredDistance(const mslam::Point &lhs, const mslam::Point &rhs) {
  const float dx = lhs.x - rhs.x;
  const float dy = lhs.y - rhs.y;
  const float dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

} // namespace

namespace mslam {

static std::vector<Voxel3> buildVoxelShifts(int adjacent_voxels) {
  std::vector<Voxel3> shifts;
  const auto side_length = static_cast<size_t>(2 * adjacent_voxels + 1);
  shifts.reserve(side_length * side_length * side_length);
  for (int dx = -adjacent_voxels; dx <= adjacent_voxels; ++dx)
    for (int dy = -adjacent_voxels; dy <= adjacent_voxels; ++dy)
      for (int dz = -adjacent_voxels; dz <= adjacent_voxels; ++dz)
        shifts.emplace_back(dx, dy, dz);
  return shifts;
}

VoxelHashMap::VoxelHashMap(float voxel_size, size_t max_points_per_voxel)
    : voxel_size_(voxel_size), inverse_voxel_size_(1.0F / voxel_size),
      max_points_per_voxel_(max_points_per_voxel), adjacent_voxels_(1),
      voxel_shifts_(buildVoxelShifts(1)) {}

PointCloud VoxelHashMap::addScan(const PointCloud &scan) {
  PointCloud added;
  added.points.reserve(scan.points.size());

  for (const auto &point : scan.points) {

    const auto voxel = PointToVoxel(point, inverse_voxel_size_);

    auto &bucket = map_[voxel];
    if (bucket.points.empty()) {
      bucket.points.reserve(max_points_per_voxel_);
    }
    if (bucket.points.size() < max_points_per_voxel_) {
      bucket.points.emplace_back(point);
      added.points.emplace_back(point);
      map_rep_.points.emplace_back(point);
    }
  }

  return added;
}

IMap::Neighbor VoxelHashMap::getClosestNeighbor(const Point &query) const {
  const auto voxel = PointToVoxel(query, inverse_voxel_size_);

  IMap::Neighbor best_neighbor{{0, 0, 0}, std::numeric_limits<float>::max()};

  for (const auto &voxel_shift : voxel_shifts_) {
    const auto query_voxel = voxel + voxel_shift;
    const auto search = map_.find(query_voxel);
    if (search == map_.end()) {
      continue;
    }

    const auto &bucket_points = search->second;
    for (const auto &point : bucket_points.points) {
      const float squared_distance = squaredDistance(point, query);
      if (squared_distance < best_neighbor.second) {
        best_neighbor = {Point{point.x, point.y, point.z}, squared_distance};
      }
    }
  }

  return best_neighbor;
}

std::vector<IMap::Neighbor>
VoxelHashMap::getClosestNNeighbors(const Point &query, int N) const {
  std::vector<IMap::Neighbor> neighbors;
  if (N <= 0) {
    return neighbors;
  }

  const auto voxel = PointToVoxel(query, inverse_voxel_size_);

  const size_t max_candidate_count =
      voxel_shifts_.size() * max_points_per_voxel_;
  neighbors.reserve(max_candidate_count);

  for (const auto &voxel_shift : voxel_shifts_) {
    const auto query_voxel = voxel + voxel_shift;
    const auto search = map_.find(query_voxel);
    if (search != map_.end()) {
      const auto &bucket_points = search->second;
      for (const auto &point : bucket_points.points) {
        const float squared_distance = squaredDistance(point, query);
        neighbors.emplace_back(Point{point.x, point.y, point.z},
                               squared_distance);
      }
    }
  }

  const auto requested_count = static_cast<std::size_t>(N);
  if (neighbors.size() <= requested_count) {
    std::sort(neighbors.begin(), neighbors.end(),
              [](const auto &lhs, const auto &rhs) {
                return lhs.second < rhs.second;
              });
    return neighbors;
  }

  std::nth_element(
      neighbors.begin(), neighbors.begin() + requested_count, neighbors.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });
  neighbors.resize(requested_count);
  std::sort(
      neighbors.begin(), neighbors.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });

  return neighbors;
}

/**
 * @brief Get a Point Cloud Representation. Copy might be made.
 *
 * @return PointCloud2D
 */
const PointCloud &VoxelHashMap::getPointCloudRepresentation() const {
  return map_rep_;
}

/**
 * @brief Set number of adjacent voxels from the query point to search for
 * bucket\s. This number exponentially decreases performance, albeit to as much
 * for low numbers.
 * @param adjacent_voxels
 */
void VoxelHashMap::clear() {
  map_.clear();
  map_rep_.points.clear();
}

void VoxelHashMap::setNumAdjacentVoxelSearch(int adjacent_voxels) {
  if (adjacent_voxels_ == adjacent_voxels) {
    return;
  }
  adjacent_voxels_ = adjacent_voxels;
  voxel_shifts_ = buildVoxelShifts(adjacent_voxels);
}

} // namespace mslam
