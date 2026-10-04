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
  added.reserve(scan.size());

  if (max_points_per_voxel_ == 0) {
    return added;
  }

  for (const auto &point : scan) {

    const auto voxel = PointToVoxel(point, inverse_voxel_size_);

    auto &bucket = map_[voxel];
    if (bucket.empty()) {
      bucket.reserve(max_points_per_voxel_);
    }
    if (bucket.size() < max_points_per_voxel_) {
      bucket.emplace_back(point);
      added.emplace_back(point);
      map_rep_dirty_ = true;
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
    for (const auto &point : bucket_points) {
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
      for (const auto &point : bucket_points) {
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
 * @brief Bound the map around the current pose.
 *
 * The common case (map already within bounds) is O(1): the range pass is only
 * executed when `max_range` is configured, and the budget pass only when the
 * voxel count exceeds `max_voxels`. Eviction keeps the voxels nearest to
 * `center`, so the map behaves like a local (sliding) map regardless of how
 * far the session has travelled.
 */
void VoxelHashMap::prune(const Point &center, float max_range,
                         size_t max_voxels) {
  const bool bounded_by_range = max_range > 0.0F;
  const bool over_budget = max_voxels > 0 && map_.size() > max_voxels;

  if (!bounded_by_range && !over_budget) {
    return;
  }

  const Eigen::Vector3d center_vec(center.x, center.y, center.z);
  const float max_range_squared =
      bounded_by_range ? max_range * max_range : 0.0F;
  const float half_voxel = 0.5F * voxel_size_;

  auto voxel_center_squared_distance = [&](const Voxel3 &voxel) {
    const Eigen::Vector3d voxel_center =
        voxel.cast<double>() * static_cast<double>(voxel_size_) +
        Eigen::Vector3d::Constant(static_cast<double>(half_voxel));
    return static_cast<float>((voxel_center - center_vec).squaredNorm());
  };

  if (over_budget) {
    // Keep the nearest voxels, with some headroom below the budget so that a
    // stationary sensor does not trigger an O(n) eviction on every scan.
    const size_t target = std::max<size_t>(1, max_voxels - max_voxels / 10);

    prune_distances_.clear();
    prune_distances_.reserve(map_.size());
    for (const auto &entry : map_) {
      prune_distances_.push_back(voxel_center_squared_distance(entry.first));
    }
    std::nth_element(prune_distances_.begin(),
                     prune_distances_.begin() + (target - 1),
                     prune_distances_.end());
    const float threshold = prune_distances_[target - 1];

    for (auto it = map_.begin(); it != map_.end();) {
      if (voxel_center_squared_distance(it->first) > threshold) {
        it = map_.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (bounded_by_range) {
    for (auto it = map_.begin(); it != map_.end();) {
      if (voxel_center_squared_distance(it->first) > max_range_squared) {
        it = map_.erase(it);
      } else {
        ++it;
      }
    }
  }

  map_rep_dirty_ = true;
}

/**
 * @brief Get a Point Cloud Representation, built lazily on request.
 *
 * @return PointCloud2D
 */
const PointCloud &VoxelHashMap::getPointCloudRepresentation() const {
  if (!map_rep_dirty_) {
    return map_rep_;
  }

  size_t total_points = 0;
  for (const auto &entry : map_) {
    total_points += entry.second.size();
  }

  map_rep_.clear();
  map_rep_.reserve(total_points);
  for (const auto &entry : map_) {
    map_rep_.insert(map_rep_.end(), entry.second.begin(), entry.second.end());
  }
  map_rep_dirty_ = false;

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
  map_rep_.clear();
  map_rep_dirty_ = true;
}

void VoxelHashMap::setNumAdjacentVoxelSearch(int adjacent_voxels) {
  if (adjacent_voxels_ == adjacent_voxels) {
    return;
  }
  adjacent_voxels_ = adjacent_voxels;
  voxel_shifts_ = buildVoxelShifts(adjacent_voxels);
}

} // namespace mslam
