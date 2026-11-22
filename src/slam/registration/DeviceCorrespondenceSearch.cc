#include "mslam/slam/registration/DeviceCorrespondenceSearch.hh"

#include "mslam/map/SyclVoxelHashMap.hh"
#include "mslam/slam/NormalMath.hh"

#include "moptim/ExecutionContext.hh"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <new>
#include <sycl/sycl.hpp>

namespace mslam {

namespace {

std::atomic<std::uint64_t> g_launches{0};

class CorrespondenceKernel;

struct KernelArgs {
  device::MapView map;
  DeviceNormalCache cache;
  const device::PointXYZI *source;
  std::size_t count;
  std::size_t chunk;
  float gate_squared;
  std::uint32_t num_neighbors;
  DeviceCorrespondence *results;
};

void processPoint(const KernelArgs &args, std::size_t index) {
  const device::PointXYZI &p = args.source[index];
  DeviceCorrespondence &out = args.results[index];
  out.matched = 0;
  out.computed = 0;
  out.has_normal = 0;

  const device::Nearest nearest =
      device::closestNeighbor(args.map, p.x, p.y, p.z);
  // CorrespondenceFinder: a point at or beyond the gate has no correspondence.
  if (!(nearest.squared_distance < args.gate_squared)) {
    return;
  }
  const device::PointXYZI &m = args.map.pool[nearest.slot];
  out.matched = 1;
  out.slot = nearest.slot;
  out.map_point[0] = m.x;
  out.map_point[1] = m.y;
  out.map_point[2] = m.z;

  const std::uint8_t state = args.cache.state[nearest.slot];
  if (state != DeviceNormalCache::kUnknown) {
    out.has_normal = state == DeviceNormalCache::kNormal ? 1 : 0;
    const double *cached = args.cache.normals + 3 * nearest.slot;
    out.normal[0] = cached[0];
    out.normal[1] = cached[1];
    out.normal[2] = cached[2];
    return;
  }

  // NormalEstimator::compute(): the map point's nearest map points.
  std::uint32_t slots[device::kMaxNeighbors];
  float distances[device::kMaxNeighbors];
  const std::uint32_t found = device::closestNeighbors(
      args.map, m.x, m.y, m.z, args.num_neighbors, slots, distances);
  Eigen::Vector3d normal;
  const bool ok = normalFromPoints(
      found,
      [&](std::size_t i) -> const device::PointXYZI & {
        return args.map.pool[slots[i]];
      },
      normal);
  out.computed = 1;
  out.has_normal = ok ? 1 : 0;
  if (ok) {
    out.normal[0] = normal.x();
    out.normal[1] = normal.y();
    out.normal[2] = normal.z();
  }
}

} // namespace

std::uint64_t deviceCorrespondenceLaunches() {
  return g_launches.load(std::memory_order_relaxed);
}

DeviceCorrespondenceSearch::DeviceCorrespondenceSearch(int num_neighbors)
    : num_neighbors_(std::clamp(num_neighbors, 0, device::kMaxNeighbors)) {}

DeviceCorrespondenceSearch::~DeviceCorrespondenceSearch() {
  if (allocated_for_ != nullptr) {
    sycl::queue &queue = allocated_for_->context()->queue();
    sycl::free(results_, queue);
    if (source_copy_ != nullptr) {
      sycl::free(source_copy_, queue);
    }
  }
}

void DeviceCorrespondenceSearch::reserve(const SyclVoxelHashMap &map,
                                         std::size_t points) {
  if (points <= capacity_ && allocated_for_ == &map) {
    return;
  }
  if (allocated_for_ != nullptr) {
    sycl::queue &old_queue = allocated_for_->context()->queue();
    sycl::free(results_, old_queue);
    if (source_copy_ != nullptr) {
      sycl::free(source_copy_, old_queue);
    }
  }
  sycl::queue &queue = map.context()->queue();
  const std::size_t capacity = std::max<std::size_t>(points, 2 * capacity_);
  results_ = sycl::malloc_shared<DeviceCorrespondence>(capacity, queue);
  source_copy_ = map.context()->hostAccessible()
                     ? nullptr
                     : sycl::malloc_shared<Point>(capacity, queue);
  if (results_ == nullptr ||
      (!map.context()->hostAccessible() && source_copy_ == nullptr)) {
    throw std::bad_alloc();
  }
  capacity_ = capacity;
  allocated_for_ = &map;
}

const DeviceCorrespondence *
DeviceCorrespondenceSearch::run(const SyclVoxelHashMap &map,
                                const PointCloud &source,
                                float max_correspondence_distance,
                                std::size_t width) {
  const std::size_t count = source.size();
  reserve(map, std::max<std::size_t>(count, 1));
  if (count == 0) {
    return results_;
  }
  const Point *points = source.data();
  if (source_copy_ != nullptr) {
    std::memcpy(source_copy_, points, count * sizeof(Point));
    points = source_copy_;
  }

  width = std::clamp<std::size_t>(width, 1, count);
  KernelArgs args;
  args.map = map.view();
  args.cache = map.normalCache();
  args.source = reinterpret_cast<const device::PointXYZI *>(points);
  args.count = count;
  args.chunk = (count + width - 1) / width;
  args.gate_squared = max_correspondence_distance * max_correspondence_distance;
  args.num_neighbors = static_cast<std::uint32_t>(num_neighbors_);
  args.results = results_;

  g_launches.fetch_add(1, std::memory_order_relaxed);
  map.context()
      ->queue()
      .parallel_for<CorrespondenceKernel>(
          sycl::nd_range<1>{width, 1},
          [args](sycl::nd_item<1> item) {
            const std::size_t begin = item.get_group_linear_id() * args.chunk;
            const std::size_t end = std::min(args.count, begin + args.chunk);
            for (std::size_t i = begin; i < end; ++i) {
              processPoint(args, i);
            }
          })
      .wait_and_throw();
  return results_;
}

} // namespace mslam
