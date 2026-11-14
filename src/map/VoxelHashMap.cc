#include "mslam/map/VoxelHashMap.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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
      voxel_shifts_(buildVoxelShifts(1)),
      home_shift_index_(voxel_shifts_.size() / 2) {}

std::uint32_t VoxelHashMap::allocateChunk() {
  if (!free_chunks_.empty()) {
    const auto offset = free_chunks_.back();
    free_chunks_.pop_back();
    return offset;
  }
  const auto offset = static_cast<std::uint32_t>(pool_.size());
  pool_.resize(pool_.size() + max_points_per_voxel_);
  return offset;
}

PointCloud VoxelHashMap::addScan(const PointCloud &scan) {
  PointCloud added;
  added.reserve(scan.size());

  if (max_points_per_voxel_ == 0) {
    return added;
  }

  for (const auto &point : scan) {

    const auto voxel = PointToVoxel(point, inverse_voxel_size_);

    auto it = map_.find(voxel);
    if (it == map_.end()) {
      it = map_.emplace(voxel, Bucket{allocateChunk(), 0}).first;
    }
    Bucket &bucket = it.value();
    if (bucket.count < max_points_per_voxel_) {
      pool_[bucket.offset + bucket.count] = point;
      ++bucket.count;
      added.emplace_back(point);
      map_rep_dirty_ = true;
    }
  }

  return added;
}

VoxelHashMap::AxisGaps
VoxelHashMap::computeAxisGaps(const Point &query, const Voxel3 &voxel) const {
  // Distance from the query to the near face of the voxel at each shift,
  // reduced by a small slack so rounding in the floor() that assigned points
  // to voxels can never make the bound exceed a real point distance.
  constexpr float kSlack = 1e-6F;
  const float q[3] = {query.x, query.y, query.z};
  AxisGaps gaps;
  for (int axis = 0; axis < 3; ++axis) {
    const float slack = kSlack * (std::abs(q[axis]) + voxel_size_);
    for (int shift = -adjacent_voxels_; shift <= adjacent_voxels_; ++shift) {
      float distance = 0.0F;
      if (shift > 0) {
        distance = static_cast<float>(voxel[axis] + shift) * voxel_size_ -
                   q[axis] - slack;
      } else if (shift < 0) {
        distance = q[axis] -
                   static_cast<float>(voxel[axis] + shift + 1) * voxel_size_ -
                   slack;
      }
      distance = std::max(distance, 0.0F);
      gaps.gap[shift + adjacent_voxels_][axis] = distance * distance;
    }
  }
  return gaps;
}

IMap::Neighbor VoxelHashMap::getClosestNeighbor(const Point &query) const {
  const auto voxel = PointToVoxel(query, inverse_voxel_size_);
  const auto gaps = computeAxisGaps(query, voxel);

  IMap::Neighbor best_neighbor{{0, 0, 0}, std::numeric_limits<float>::max()};
  // Ties between equal distances resolve to the lowest shift index, which is
  // what an exhaustive scan of voxel_shifts_ in order returns, even though the
  // home voxel is visited first.
  size_t best_shift = std::numeric_limits<size_t>::max();

  const auto visit = [&](size_t shift_index) {
    const Voxel3 &shift = voxel_shifts_[shift_index];
    const auto search = map_.find(voxel + shift);
    if (search == map_.end()) {
      return;
    }
    const Bucket bucket = search->second;
    const Point *points = pool_.data() + bucket.offset;
    for (std::uint32_t i = 0; i < bucket.count; ++i) {
      const float squared_distance = squaredDistance(points[i], query);
      if (squared_distance < best_neighbor.second ||
          (squared_distance == best_neighbor.second &&
           shift_index < best_shift)) {
        best_neighbor = {Point{points[i].x, points[i].y, points[i].z},
                         squared_distance};
        best_shift = shift_index;
      }
    }
  };

  visit(home_shift_index_);

  for (size_t index = 0; index < voxel_shifts_.size(); ++index) {
    if (index == home_shift_index_) {
      continue;
    }
    const Voxel3 &shift = voxel_shifts_[index];
    const float lower_bound = gaps.gap[shift.x() + adjacent_voxels_][0] +
                              gaps.gap[shift.y() + adjacent_voxels_][1] +
                              gaps.gap[shift.z() + adjacent_voxels_][2];
    if (lower_bound > best_neighbor.second) {
      continue;
    }
    visit(index);
  }

  return best_neighbor;
}

std::size_t VoxelHashMap::findClosestNeighbors(const Point &query,
                                               std::span<Neighbor> out) const {
  const std::size_t capacity = out.size();
  if (capacity == 0) {
    return 0;
  }

  const auto voxel = PointToVoxel(query, inverse_voxel_size_);
  const auto gaps = computeAxisGaps(query, voxel);

  // `out` is kept sorted by distance; a candidate is inserted by shifting the
  // farther entries up. Equal distances keep their visiting order.
  std::size_t count = 0;

  const auto visit = [&](size_t shift_index) {
    const auto search = map_.find(voxel + voxel_shifts_[shift_index]);
    if (search == map_.end()) {
      return;
    }
    const Bucket bucket = search->second;
    const Point *points = pool_.data() + bucket.offset;
    for (std::uint32_t i = 0; i < bucket.count; ++i) {
      const float squared_distance = squaredDistance(points[i], query);
      if (count == capacity && squared_distance >= out[count - 1].second) {
        continue;
      }
      std::size_t position = count < capacity ? count : capacity - 1;
      while (position > 0 && out[position - 1].second > squared_distance) {
        out[position] = out[position - 1];
        --position;
      }
      out[position] = {Point{points[i].x, points[i].y, points[i].z},
                       squared_distance};
      if (count < capacity) {
        ++count;
      }
    }
  };

  visit(home_shift_index_);

  for (size_t index = 0; index < voxel_shifts_.size(); ++index) {
    if (index == home_shift_index_) {
      continue;
    }
    if (count == capacity) {
      const Voxel3 &shift = voxel_shifts_[index];
      const float lower_bound = gaps.gap[shift.x() + adjacent_voxels_][0] +
                                gaps.gap[shift.y() + adjacent_voxels_][1] +
                                gaps.gap[shift.z() + adjacent_voxels_][2];
      if (lower_bound > out[count - 1].second) {
        continue;
      }
    }
    visit(index);
  }

  return count;
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
        releaseChunk(it->second.offset);
        it = map_.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (bounded_by_range) {
    for (auto it = map_.begin(); it != map_.end();) {
      if (voxel_center_squared_distance(it->first) > max_range_squared) {
        releaseChunk(it->second.offset);
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
    total_points += entry.second.count;
  }

  map_rep_.clear();
  map_rep_.reserve(total_points);
  for (const auto &entry : map_) {
    const auto first = pool_.begin() + entry.second.offset;
    map_rep_.insert(map_rep_.end(), first, first + entry.second.count);
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
  pool_.clear();
  free_chunks_.clear();
  map_rep_.clear();
  map_rep_dirty_ = true;
}

void VoxelHashMap::setNumAdjacentVoxelSearch(int adjacent_voxels) {
  if (adjacent_voxels < 0 || adjacent_voxels > kMaxAdjacentVoxels) {
    throw std::invalid_argument(
        "Adjacent voxel search radius must be in [0, kMaxAdjacentVoxels]");
  }
  if (adjacent_voxels_ == adjacent_voxels) {
    return;
  }
  adjacent_voxels_ = adjacent_voxels;
  voxel_shifts_ = buildVoxelShifts(adjacent_voxels);
  home_shift_index_ = voxel_shifts_.size() / 2;
}

} // namespace mslam
