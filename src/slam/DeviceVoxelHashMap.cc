#include "mslam/map/DeviceVoxelHashMap.hh"

#include "moptim/ExecutionContext.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace mslam {

static_assert(sizeof(Point) == sizeof(device::PointXYZI) &&
                  alignof(Point) == alignof(device::PointXYZI),
              "the mirror stores mslam::Point as device::PointXYZI");

namespace {

constexpr std::uint32_t kInitialCapacity = 1U << 12;

/// Memory the host and the kernels of @p context share (shared USM on SYCL,
/// plain memory on the host backend).
template <class T>
T *allocate(moptim::ExecutionContext &context, std::size_t count) {
  return static_cast<T *>(context.allocateShared(
      std::max<std::size_t>(count, 1) * sizeof(T), alignof(T) < 64 ? 64 : alignof(T)));
}

template <class T>
void release(moptim::ExecutionContext &context, T *&data) {
  context.deallocate(data);
  data = nullptr;
}

bool sameKey(const device::VoxelKey &a, const device::VoxelKey &b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

} // namespace

DeviceVoxelHashMap::DeviceVoxelHashMap(
    float voxel_size, size_t max_points_per_voxel,
    std::shared_ptr<moptim::ExecutionContext> context)
    : VoxelHashMap(voxel_size, max_points_per_voxel),
      context_(context ? std::move(context) : moptim::defaultContext()) {
  growTable(kInitialCapacity);
}

DeviceVoxelHashMap::~DeviceVoxelHashMap() {
  moptim::ExecutionContext &queue = *context_;
  release(queue, keys_);
  release(queue, buckets_);
  release(queue, pool_mirror_);
  release(queue, normal_state_);
  release(queue, normal_values_);
  release(queue, shifts_);
}

PointCloud DeviceVoxelHashMap::addScan(const PointCloud &scan) {
  PointCloud added = VoxelHashMap::addScan(scan);
  const float inverse_voxel_size = 1.0F / voxelSize();
  changed_.clear();
  for (const Point &point : added) {
    const device::VoxelKey key =
        device::voxelOf(point.x, point.y, point.z, inverse_voxel_size);
    [[maybe_unused]] const bool inserted = mirrorInsert(key, point);
    // The host map accepted the point, so its voxel had room: the mirror's
    // voxel holds the same points.
    assert(inserted);
    if (changed_.empty() || !sameKey(changed_.back(), key)) {
      changed_.push_back(key);
    }
  }
  std::sort(changed_.begin(), changed_.end(),
            [](const device::VoxelKey &a, const device::VoxelKey &b) {
              return a.x != b.x ? a.x < b.x : a.y != b.y ? a.y < b.y : a.z < b.z;
            });
  changed_.erase(std::unique(changed_.begin(), changed_.end(), sameKey),
                 changed_.end());
  // A map that has cached nothing yet (one being built before it is searched,
  // like a loop-closure target) has nothing to invalidate.
  if (known_normals_ > 0) {
    invalidateAround(changed_);
  }
  return added;
}

void DeviceVoxelHashMap::prune(const Point &center, float max_range,
                             size_t max_voxels) {
  // prune() only removes voxels, so an unchanged count means nothing changed.
  const size_t before = size();
  VoxelHashMap::prune(center, max_range, max_voxels);
  if (size() != before) {
    rebuild();
  }
}

void DeviceVoxelHashMap::clear() {
  VoxelHashMap::clear();
  rebuild();
}

device::MapView DeviceVoxelHashMap::view() const {
  updateShifts();
  device::MapView view;
  view.keys = keys_;
  view.buckets = buckets_;
  view.mask = capacity_ - 1;
  view.pool = pool_mirror_;
  view.voxel_size = voxelSize();
  view.inverse_voxel_size = 1.0F / voxelSize();
  view.adjacent_voxels = numAdjacentVoxelSearch();
  view.shifts = shifts_;
  view.num_shifts = num_shifts_;
  view.home_shift = num_shifts_ / 2;
  return view;
}

void DeviceVoxelHashMap::storeNormal(std::uint32_t slot,
                                   const double *normal) const {
  if (normal_state_[slot] == DeviceNormalCache::kUnknown) {
    ++known_normals_;
  }
  if (normal == nullptr) {
    normal_state_[slot] = DeviceNormalCache::kNoNormal;
    return;
  }
  std::memcpy(normal_values_ + 3 * static_cast<std::size_t>(slot), normal,
              3 * sizeof(double));
  normal_state_[slot] = DeviceNormalCache::kNormal;
}

std::size_t DeviceVoxelHashMap::cachedNormals() const {
  return static_cast<std::size_t>(std::count_if(
      normal_state_, normal_state_ + pool_used_, [](std::uint8_t state) {
        return state != DeviceNormalCache::kUnknown;
      }));
}

bool DeviceVoxelHashMap::mirrorInsert(const device::VoxelKey &key,
                                    const Point &point) {
  const std::uint32_t slot = findOrInsertSlot(key);
  device::Bucket &bucket = buckets_[slot];
  if (bucket.count >= maxPointsPerVoxel()) {
    return false;
  }
  const std::size_t index = bucket.offset + bucket.count;
  std::memcpy(pool_mirror_ + index, &point, sizeof(Point));
  normal_state_[index] = DeviceNormalCache::kUnknown;
  ++bucket.count;
  return true;
}

std::uint32_t
DeviceVoxelHashMap::findOrInsertSlot(const device::VoxelKey &key) {
  if (2 * (static_cast<std::uint64_t>(occupied_) + 1) > capacity_) {
    growTable(2 * capacity_);
  }
  const std::uint32_t mask = capacity_ - 1;
  std::uint32_t slot =
      static_cast<std::uint32_t>(device::hashVoxel(key.x, key.y, key.z)) & mask;
  while (keys_[slot].x != device::kEmptyKey) {
    if (sameKey(keys_[slot], key)) {
      return slot;
    }
    slot = (slot + 1) & mask;
  }
  keys_[slot] = key;
  const std::size_t chunk = maxPointsPerVoxel();
  if (pool_used_ + chunk > pool_capacity_) {
    growPool(pool_used_ + chunk);
  }
  buckets_[slot] = {static_cast<std::uint32_t>(pool_used_), 0};
  pool_used_ += chunk;
  ++occupied_;
  return slot;
}

void DeviceVoxelHashMap::growTable(std::uint32_t capacity) {
  moptim::ExecutionContext &queue = *context_;
  auto *keys = allocate<device::VoxelKey>(queue, capacity);
  auto *buckets = allocate<device::Bucket>(queue, capacity);
  for (std::uint32_t i = 0; i < capacity; ++i) {
    keys[i].x = device::kEmptyKey;
  }
  const std::uint32_t mask = capacity - 1;
  for (std::uint32_t old = 0; old < capacity_; ++old) {
    if (keys_[old].x == device::kEmptyKey) {
      continue;
    }
    const device::VoxelKey &key = keys_[old];
    std::uint32_t slot =
        static_cast<std::uint32_t>(device::hashVoxel(key.x, key.y, key.z)) &
        mask;
    while (keys[slot].x != device::kEmptyKey) {
      slot = (slot + 1) & mask;
    }
    keys[slot] = key;
    buckets[slot] = buckets_[old];
  }
  release(queue, keys_);
  release(queue, buckets_);
  keys_ = keys;
  buckets_ = buckets;
  capacity_ = capacity;
}

void DeviceVoxelHashMap::growPool(std::size_t slots) {
  moptim::ExecutionContext &queue = *context_;
  const std::size_t capacity = std::max(slots, 2 * pool_capacity_);
  auto *pool = allocate<device::PointXYZI>(queue, capacity);
  auto *state = allocate<std::uint8_t>(queue, capacity);
  auto *values = allocate<double>(queue, 3 * capacity);
  if (pool_used_ > 0) {
    std::memcpy(pool, pool_mirror_, pool_used_ * sizeof(device::PointXYZI));
    std::memcpy(state, normal_state_, pool_used_);
    std::memcpy(values, normal_values_, 3 * pool_used_ * sizeof(double));
  }
  std::fill(state + pool_used_, state + capacity, DeviceNormalCache::kUnknown);
  release(queue, pool_mirror_);
  release(queue, normal_state_);
  release(queue, normal_values_);
  pool_mirror_ = pool;
  normal_state_ = state;
  normal_values_ = values;
  pool_capacity_ = capacity;
}

void DeviceVoxelHashMap::invalidateAround(
    const std::vector<device::VoxelKey> &changed) {
  // The normal of a point in voxel u reads the voxels within the search radius
  // of u, so a change in a voxel invalidates the points of every voxel within
  // that radius of it. The table is far larger than the caches and the
  // neighbourhoods are random accesses into it, so all their home slots are
  // computed and prefetched first, then probed: the misses overlap.
  const device::MapView map = view();
  probes_.clear();
  for (const device::VoxelKey &key : changed) {
    for (std::uint32_t s = 0; s < map.num_shifts; ++s) {
      const device::VoxelKey &shift = map.shifts[s];
      const device::VoxelKey neighbour{key.x - shift.x, key.y - shift.y,
                                       key.z - shift.z};
      const std::uint32_t home =
          static_cast<std::uint32_t>(
              device::hashVoxel(neighbour.x, neighbour.y, neighbour.z)) &
          map.mask;
      __builtin_prefetch(keys_ + home);
      probes_.push_back({neighbour, home});
    }
  }
  for (const Probe &probe : probes_) {
    std::uint32_t slot = probe.home;
    while (keys_[slot].x != device::kEmptyKey) {
      if (sameKey(keys_[slot], probe.key)) {
        const device::Bucket bucket = buckets_[slot];
        for (std::uint32_t i = 0; i < bucket.count; ++i) {
          std::uint8_t &state = normal_state_[bucket.offset + i];
          if (state != DeviceNormalCache::kUnknown) {
            state = DeviceNormalCache::kUnknown;
            --known_normals_;
          }
        }
        break;
      }
      slot = (slot + 1) & map.mask;
    }
  }
}

void DeviceVoxelHashMap::rebuild() {
  for (std::uint32_t i = 0; i < capacity_; ++i) {
    keys_[i].x = device::kEmptyKey;
  }
  occupied_ = 0;
  pool_used_ = 0;
  forEachVoxel([&](const Voxel3 &voxel, const Point *points, size_t count) {
    const device::VoxelKey key{voxel.x(), voxel.y(), voxel.z()};
    for (size_t i = 0; i < count; ++i) {
      mirrorInsert(key, points[i]);
    }
  });
  std::fill(normal_state_, normal_state_ + pool_capacity_,
            DeviceNormalCache::kUnknown);
  known_normals_ = 0;
}

void DeviceVoxelHashMap::updateShifts() const {
  const int radius = numAdjacentVoxelSearch();
  if (radius == shifts_radius_) {
    return;
  }
  moptim::ExecutionContext &queue = *context_;
  const std::uint32_t side = static_cast<std::uint32_t>(2 * radius + 1);
  release(queue, shifts_);
  shifts_ = allocate<device::VoxelKey>(queue, side * side * side);
  std::uint32_t n = 0;
  // VoxelHashMap's buildVoxelShifts() order: x outer, z inner.
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        shifts_[n++] = {dx, dy, dz};
      }
    }
  }
  num_shifts_ = n;
  if (shifts_radius_ != -1) {
    // Cached normals were invalidated for another radius: drop them all.
    std::fill(normal_state_, normal_state_ + pool_capacity_,
              DeviceNormalCache::kUnknown);
    known_normals_ = 0;
  }
  shifts_radius_ = radius;
}

} // namespace mslam
