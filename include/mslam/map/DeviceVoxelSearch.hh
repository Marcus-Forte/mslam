#pragma once

// Voxel map searches that run in SYCL kernels (and on the host): a plain view of
// a voxel hash table in USM and the 1-NN / k-NN searches of VoxelHashMap, with
// the same arithmetic, visiting order, pruning and tie-breaking, so that a search
// in the view returns what VoxelHashMap::getClosestNeighbor() /
// findClosestNeighbors() return on the same map. No Eigen, no virtual calls, no
// allocation: raw pointers and fixed-size arrays only.
#include <cstddef>
#include <cstdint>
#include <limits>

#ifdef __SYCL_DEVICE_ONLY__
#include <sycl/sycl.hpp>
#else
#include <cmath>
#endif

namespace mslam::device {

/// Largest search radius in voxels (VoxelHashMap::kMaxAdjacentVoxels).
inline constexpr int kMaxAdjacentVoxels = 8;
/// Largest number of neighbours of a k-NN search (NormalEstimator::kMaxNeighbors).
inline constexpr int kMaxNeighbors = 16;

/// A voxel key; `x == kEmptyKey` marks an empty table slot.
struct VoxelKey {
  std::int32_t x, y, z;
};
inline constexpr std::int32_t kEmptyKey = std::numeric_limits<std::int32_t>::min();

/// The points of a voxel: `count` points from `offset` on in the pool.
struct Bucket {
  std::uint32_t offset;
  std::uint32_t count;
};

/// Same layout as mslam::Point (msensor::Point3I).
struct PointXYZI {
  float x, y, z, intensity;
};

/// Everything a kernel needs to search the map. Trivially copyable: a kernel
/// captures it by value. The arrays live in USM.
struct MapView {
  const VoxelKey *keys = nullptr;   ///< `mask + 1` slots, open addressing.
  const Bucket *buckets = nullptr;  ///< Parallel to `keys`.
  std::uint32_t mask = 0;           ///< Capacity - 1 (a power of two).
  const PointXYZI *pool = nullptr;  ///< The points of all voxels.
  float voxel_size = 0.0F;
  float inverse_voxel_size = 0.0F;
  std::int32_t adjacent_voxels = 1;
  const VoxelKey *shifts = nullptr; ///< (2r+1)^3 shifts, in VoxelHashMap order.
  std::uint32_t num_shifts = 0;
  std::uint32_t home_shift = 0;     ///< Index of the zero shift.
};

inline float floorOf(float value) {
#ifdef __SYCL_DEVICE_ONLY__
  return sycl::floor(value);
#else
  return std::floor(value);
#endif
}

inline float absOf(float value) {
#ifdef __SYCL_DEVICE_ONLY__
  return sycl::fabs(value);
#else
  return std::abs(value);
#endif
}

/// PointToVoxel().
inline VoxelKey voxelOf(float x, float y, float z, float inverse_voxel_size) {
  return {static_cast<std::int32_t>(floorOf(x * inverse_voxel_size)),
          static_cast<std::int32_t>(floorOf(y * inverse_voxel_size)),
          static_cast<std::int32_t>(floorOf(z * inverse_voxel_size))};
}

/// std::hash<Voxel3> (Voxel.hh).
inline std::uint64_t hashVoxel(std::int32_t x, std::int32_t y, std::int32_t z) {
  const std::uint64_t xy =
      static_cast<std::uint32_t>(x) |
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) << 32);
  std::uint64_t h = xy * 0x9E3779B97F4A7C15ULL;
  h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(z)) *
       0xC2B2AE3D27D4EB4FULL;
  h ^= h >> 32;
  h *= 0xD6E8FEB86659FD93ULL;
  h ^= h >> 32;
  return h;
}

/// Table slot of voxel (x, y, z), or -1 if the map has no such voxel.
inline std::int64_t findSlot(const MapView &map, std::int32_t x, std::int32_t y,
                             std::int32_t z) {
  if (map.keys == nullptr) {
    return -1;
  }
  std::uint32_t slot =
      static_cast<std::uint32_t>(hashVoxel(x, y, z)) & map.mask;
  for (std::uint32_t probe = 0; probe <= map.mask; ++probe) {
    const VoxelKey &key = map.keys[slot];
    if (key.x == kEmptyKey) {
      return -1;
    }
    if (key.x == x && key.y == y && key.z == z) {
      return slot;
    }
    slot = (slot + 1) & map.mask;
  }
  return -1;
}

/// VoxelHashMap's squaredDistance().
inline float squaredDistance(const PointXYZI &lhs, float qx, float qy,
                             float qz) {
  const float dx = lhs.x - qx;
  const float dy = lhs.y - qy;
  const float dz = lhs.z - qz;
  return dx * dx + dy * dy + dz * dz;
}

/// VoxelHashMap::computeAxisGaps(): squared lower bounds, per axis, between the
/// query and the voxels of the neighbourhood along that axis.
struct AxisGaps {
  float gap[2 * kMaxAdjacentVoxels + 1][3];
};

inline AxisGaps computeAxisGaps(const MapView &map, const float q[3],
                                const VoxelKey &voxel) {
  constexpr float kSlack = 1e-6F;
  const std::int32_t v[3] = {voxel.x, voxel.y, voxel.z};
  const std::int32_t r = map.adjacent_voxels;
  AxisGaps gaps;
  for (int axis = 0; axis < 3; ++axis) {
    const float slack = kSlack * (absOf(q[axis]) + map.voxel_size);
    for (int shift = -r; shift <= r; ++shift) {
      float distance = 0.0F;
      if (shift > 0) {
        distance = static_cast<float>(v[axis] + shift) * map.voxel_size -
                   q[axis] - slack;
      } else if (shift < 0) {
        distance = q[axis] -
                   static_cast<float>(v[axis] + shift + 1) * map.voxel_size -
                   slack;
      }
      distance = distance > 0.0F ? distance : 0.0F;
      gaps.gap[shift + r][axis] = distance * distance;
    }
  }
  return gaps;
}

inline float lowerBound(const MapView &map, const AxisGaps &gaps,
                        const VoxelKey &shift) {
  const std::int32_t r = map.adjacent_voxels;
  return gaps.gap[shift.x + r][0] + gaps.gap[shift.y + r][1] +
         gaps.gap[shift.z + r][2];
}

/// Result of a 1-NN search: the pool index of the nearest point and its squared
/// distance; `slot` is UINT32_MAX and `squared_distance` the float maximum when
/// the neighbourhood is empty (VoxelHashMap returns {0,0,0} then).
struct Nearest {
  std::uint32_t slot = std::numeric_limits<std::uint32_t>::max();
  float squared_distance = std::numeric_limits<float>::max();
};

/// VoxelHashMap::getClosestNeighbor().
inline Nearest closestNeighbor(const MapView &map, float qx, float qy,
                               float qz) {
  const float q[3] = {qx, qy, qz};
  const VoxelKey voxel = voxelOf(qx, qy, qz, map.inverse_voxel_size);
  const AxisGaps gaps = computeAxisGaps(map, q, voxel);

  Nearest best;
  std::uint32_t best_shift = std::numeric_limits<std::uint32_t>::max();
  const auto visit = [&](std::uint32_t shift_index) {
    const VoxelKey &shift = map.shifts[shift_index];
    const std::int64_t slot =
        findSlot(map, voxel.x + shift.x, voxel.y + shift.y, voxel.z + shift.z);
    if (slot < 0) {
      return;
    }
    const Bucket bucket = map.buckets[slot];
    for (std::uint32_t i = 0; i < bucket.count; ++i) {
      const float d = squaredDistance(map.pool[bucket.offset + i], qx, qy, qz);
      if (d < best.squared_distance ||
          (d == best.squared_distance && shift_index < best_shift)) {
        best.slot = bucket.offset + i;
        best.squared_distance = d;
        best_shift = shift_index;
      }
    }
  };

  visit(map.home_shift);
  for (std::uint32_t index = 0; index < map.num_shifts; ++index) {
    if (index == map.home_shift) {
      continue;
    }
    if (lowerBound(map, gaps, map.shifts[index]) > best.squared_distance) {
      continue;
    }
    visit(index);
  }
  return best;
}

/// VoxelHashMap::findClosestNeighbors() with `capacity` (<= kMaxNeighbors)
/// neighbours: pool indexes and squared distances, nearest first. Returns the
/// number written.
inline std::uint32_t closestNeighbors(const MapView &map, float qx, float qy,
                                      float qz, std::uint32_t capacity,
                                      std::uint32_t *slots, float *distances) {
  if (capacity == 0) {
    return 0;
  }
  const float q[3] = {qx, qy, qz};
  const VoxelKey voxel = voxelOf(qx, qy, qz, map.inverse_voxel_size);
  const AxisGaps gaps = computeAxisGaps(map, q, voxel);

  std::uint32_t count = 0;
  const auto visit = [&](std::uint32_t shift_index) {
    const VoxelKey &shift = map.shifts[shift_index];
    const std::int64_t slot =
        findSlot(map, voxel.x + shift.x, voxel.y + shift.y, voxel.z + shift.z);
    if (slot < 0) {
      return;
    }
    const Bucket bucket = map.buckets[slot];
    for (std::uint32_t i = 0; i < bucket.count; ++i) {
      const float d = squaredDistance(map.pool[bucket.offset + i], qx, qy, qz);
      if (count == capacity && d >= distances[count - 1]) {
        continue;
      }
      std::uint32_t position = count < capacity ? count : capacity - 1;
      while (position > 0 && distances[position - 1] > d) {
        distances[position] = distances[position - 1];
        slots[position] = slots[position - 1];
        --position;
      }
      distances[position] = d;
      slots[position] = bucket.offset + i;
      if (count < capacity) {
        ++count;
      }
    }
  };

  visit(map.home_shift);
  for (std::uint32_t index = 0; index < map.num_shifts; ++index) {
    if (index == map.home_shift) {
      continue;
    }
    if (count == capacity &&
        lowerBound(map, gaps, map.shifts[index]) > distances[count - 1]) {
      continue;
    }
    visit(index);
  }
  return count;
}

} // namespace mslam::device
