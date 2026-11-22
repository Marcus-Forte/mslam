#include "mslam/slam/DevicePreprocessor.hh"

#include "mslam/map/DeviceVoxelSearch.hh"
#include "mslam/slam/DeskewMath.hh"

#include "moptim/ExecutionContext.hh"

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <limits>
#include <new>
#include <sycl/sycl.hpp>

namespace mslam {

namespace {

std::atomic<std::uint64_t> g_launches{0};

class PreprocessKernel;

constexpr std::uint32_t kNoSlot = std::numeric_limits<std::uint32_t>::max();
constexpr double kNanosecondsPerSecond = 1e9;
constexpr std::int32_t kCoordinateOffset = 32768; ///< 16-bit voxel coordinates.

struct Args {
  const device::PointXYZI *in;
  device::PointXYZI *out;
  std::uint32_t *slots;
  std::size_t count;
  std::size_t blocks;
  std::size_t blocks_per_group;
  // Deskew.
  bool deskew;
  double start_ns;
  double step_ns;
  const double *boundaries;
  const double *omegas;
  const double *orientations;
  std::uint32_t intervals;
  // Filter.
  bool range_test;
  float min_distance_squared;
  float inverse_voxel_size;
  std::uint64_t *keys;
  std::uint64_t *values;
  std::uint32_t mask;
  std::uint64_t key_epoch;   ///< Epoch in the key's top 16 bits.
  std::uint64_t value_epoch; ///< 0xFFFFFFFF - epoch, in the value's top 32 bits.
  std::uint32_t *overflow;
};

/// Index of the IMU interval of time @p t: the last boundary <= t (interval 0
/// starts at the scan start, so deskewInto()'s `samples[k] <= t` rule).
std::uint32_t intervalOf(const Args &args, double t) {
  std::uint32_t j = 0;
  while (j + 1 < args.intervals && args.boundaries[j + 1] <= t) {
    ++j;
  }
  return j;
}

Eigen::Vector3d omegaOf(const Args &args, std::uint32_t j) {
  return {args.omegas[3 * j], args.omegas[3 * j + 1], args.omegas[3 * j + 2]};
}

void insert(const Args &args, std::size_t index,
            const device::PointXYZI &point) {
  const device::VoxelKey voxel =
      device::voxelOf(point.x, point.y, point.z, args.inverse_voxel_size);
  const std::int32_t c[3] = {voxel.x + kCoordinateOffset,
                             voxel.y + kCoordinateOffset,
                             voxel.z + kCoordinateOffset};
  for (const std::int32_t v : c) {
    if (v < 0 || v > 0xFFFF) {
      sycl::atomic_ref<std::uint32_t, sycl::memory_order::relaxed,
                       sycl::memory_scope::device>(*args.overflow)
          .store(1);
      args.slots[index] = kNoSlot;
      return;
    }
  }
  const std::uint64_t key = (args.key_epoch << 48) |
                            (static_cast<std::uint64_t>(c[0]) << 32) |
                            (static_cast<std::uint64_t>(c[1]) << 16) |
                            static_cast<std::uint64_t>(c[2]);
  std::uint32_t slot =
      static_cast<std::uint32_t>(device::hashVoxel(voxel.x, voxel.y, voxel.z)) &
      args.mask;
  for (;;) {
    sycl::atomic_ref<std::uint64_t, sycl::memory_order::relaxed,
                     sycl::memory_scope::device>
        stored(args.keys[slot]);
    std::uint64_t current = stored.load();
    if ((current >> 48) != args.key_epoch) {
      // An empty slot (a key of an earlier scan): claim it, or see who did.
      if (!stored.compare_exchange_strong(current, key)) {
        continue;
      }
      break;
    }
    if (current == key) {
      break;
    }
    slot = (slot + 1) & args.mask;
  }
  sycl::atomic_ref<std::uint64_t, sycl::memory_order::relaxed,
                   sycl::memory_scope::device>(args.values[slot])
      .fetch_min((args.value_epoch << 32) | static_cast<std::uint64_t>(index));
  args.slots[index] = slot;
}

void processBlock(const Args &args, std::size_t block) {
  const std::size_t first = block * DevicePreprocessor::kBlock;
  const std::size_t last =
      std::min(args.count, first + DevicePreprocessor::kBlock);

  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  double previous_time_ns = 0.0;
  std::uint32_t interval = 0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  deskew_detail::RotationIncrementMemo increments;
  if (args.deskew) {
    // The orientation at the block's first point: its interval's start
    // orientation advanced to the point.
    const double t = args.start_ns + static_cast<double>(first) * args.step_ns;
    interval = intervalOf(args, t);
    angular_velocity = omegaOf(args, interval);
    const Eigen::Map<const Eigen::Matrix3d> start(args.orientations +
                                                  9 * interval);
    orientation = start * deskew_detail::rotationIncrement(
                              angular_velocity,
                              (t - args.boundaries[interval]) /
                                  kNanosecondsPerSecond);
    previous_time_ns = t;
  }

  for (std::size_t i = first; i < last; ++i) {
    const device::PointXYZI &point = args.in[i];
    device::PointXYZI corrected_point = point;
    if (args.deskew) {
      // deskewInto()'s loop.
      const double point_time_ns =
          args.start_ns + static_cast<double>(i) * args.step_ns;
      while (interval + 1 < args.intervals &&
             args.boundaries[interval + 1] <= point_time_ns) {
        const double boundary_ns = args.boundaries[interval + 1];
        orientation *= increments.get(angular_velocity,
                                      (boundary_ns - previous_time_ns) /
                                          kNanosecondsPerSecond);
        previous_time_ns = boundary_ns;
        ++interval;
        angular_velocity = omegaOf(args, interval);
        increments.reset();
      }
      orientation *= increments.get(angular_velocity,
                                    (point_time_ns - previous_time_ns) /
                                        kNanosecondsPerSecond);
      previous_time_ns = point_time_ns;
      const Eigen::Vector3d corrected =
          orientation * Eigen::Vector3d(point.x, point.y, point.z);
      corrected_point.x = static_cast<float>(corrected.x());
      corrected_point.y = static_cast<float>(corrected.y());
      corrected_point.z = static_cast<float>(corrected.z());
    }
    args.out[i] = corrected_point;

    // VoxelFilter's range test, on the deskewed point.
    const float squared = corrected_point.x * corrected_point.x +
                          corrected_point.y * corrected_point.y +
                          corrected_point.z * corrected_point.z;
    if (args.range_test && !(squared >= args.min_distance_squared)) {
      args.slots[i] = kNoSlot;
      continue;
    }
    insert(args, i, corrected_point);
  }
}

template <class T> T *allocate(sycl::queue &queue, std::size_t count) {
  T *data = sycl::malloc_shared<T>(std::max<std::size_t>(count, 1), queue);
  if (data == nullptr) {
    throw std::bad_alloc();
  }
  return data;
}

template <class T> void release(sycl::queue &queue, T *&data) {
  if (data != nullptr) {
    sycl::free(data, queue);
    data = nullptr;
  }
}

} // namespace

std::uint64_t devicePreprocessLaunches() {
  return g_launches.load(std::memory_order_relaxed);
}

DevicePreprocessor::DevicePreprocessor(
    std::shared_ptr<moptim::ExecutionContext> context)
    : context_(context ? std::move(context) : moptim::defaultContext()) {
  overflow_ = allocate<std::uint32_t>(context_->queue(), 1);
}

DevicePreprocessor::~DevicePreprocessor() {
  sycl::queue &queue = context_->queue();
  release(queue, input_);
  release(queue, output_);
  release(queue, slots_);
  release(queue, keys_);
  release(queue, values_);
  release(queue, boundaries_);
  release(queue, omegas_);
  release(queue, orientations_);
  release(queue, overflow_);
}

const Point *DevicePreprocessor::deskewed() const {
  return reinterpret_cast<const Point *>(output_);
}

void DevicePreprocessor::reserve(std::size_t points, std::size_t intervals) {
  sycl::queue &queue = context_->queue();
  if (points > capacity_) {
    const std::size_t capacity = std::max(points, 2 * capacity_);
    release(queue, input_);
    release(queue, output_);
    release(queue, slots_);
    if (!context_->hostAccessible()) {
      input_ = allocate<Point>(queue, capacity);
    }
    output_ = allocate<Point>(queue, capacity);
    slots_ = allocate<std::uint32_t>(queue, capacity);
    capacity_ = capacity;
  }
  std::size_t table = 1;
  while (table < 2 * points) {
    table <<= 1;
  }
  if (table > table_capacity_) {
    release(queue, keys_);
    release(queue, values_);
    keys_ = allocate<std::uint64_t>(queue, table);
    values_ = allocate<std::uint64_t>(queue, table);
    table_capacity_ = table;
    epoch_ = 0; // fresh table: restart the epochs
    std::fill(keys_, keys_ + table, std::uint64_t{0});
    std::fill(values_, values_ + table,
              std::numeric_limits<std::uint64_t>::max());
  }
  if (intervals > interval_capacity_) {
    const std::size_t capacity = std::max<std::size_t>(intervals, 64);
    release(queue, boundaries_);
    release(queue, omegas_);
    release(queue, orientations_);
    boundaries_ = allocate<double>(queue, capacity);
    omegas_ = allocate<double>(queue, 3 * capacity);
    orientations_ = allocate<double>(queue, 9 * capacity);
    interval_capacity_ = capacity;
  }
}

void DevicePreprocessor::nextEpoch() {
  // Keys keep 16 bits of epoch: reset the table before they wrap.
  if (++epoch_ > 0xFFFF) {
    std::fill(keys_, keys_ + table_capacity_, std::uint64_t{0});
    std::fill(values_, values_ + table_capacity_,
              std::numeric_limits<std::uint64_t>::max());
    epoch_ = 1;
  }
}

bool DevicePreprocessor::process(
    const Scan &scan, const std::vector<msensor::IMUData> &imu_samples,
    double delta_t, float min_distance, float voxel_size, float min_intensity,
    PointCloud &out) {
  const std::size_t count = scan.points.size();
  const bool deskew = delta_t > 0.0 &&
                      deskewIntervals(scan, imu_samples, delta_t, intervals_);
  last_deskewed_ = deskew;
  reserve(count, intervals_.boundary_ns.size());
  nextEpoch();
  out.clear();
  if (count == 0) {
    return true;
  }

  const Point *input = scan.points.data();
  if (input_ != nullptr) {
    std::memcpy(input_, input, count * sizeof(Point));
    input = input_;
  }
  const std::size_t intervals = deskew ? intervals_.boundary_ns.size() : 0;
  for (std::size_t j = 0; j < intervals; ++j) {
    boundaries_[j] = intervals_.boundary_ns[j];
    std::memcpy(omegas_ + 3 * j, intervals_.omega[j].data(), 3 * sizeof(double));
    std::memcpy(orientations_ + 9 * j, intervals_.orientation[j].data(),
                9 * sizeof(double));
  }
  *overflow_ = 0;

  Args args;
  args.in = reinterpret_cast<const device::PointXYZI *>(input);
  args.out = reinterpret_cast<device::PointXYZI *>(output_);
  args.slots = slots_;
  args.count = count;
  args.blocks = (count + kBlock - 1) / kBlock;
  const std::size_t width = std::min(width_, args.blocks);
  args.blocks_per_group = (args.blocks + width - 1) / width;
  args.deskew = deskew;
  args.start_ns = intervals_.start_ns;
  args.step_ns = intervals_.step_ns;
  args.boundaries = boundaries_;
  args.omegas = omegas_;
  args.orientations = orientations_;
  args.intervals = static_cast<std::uint32_t>(intervals);
  args.range_test = min_distance >= 0.0F;
  args.min_distance_squared = min_distance * min_distance;
  args.inverse_voxel_size = 1.0F / voxel_size;
  args.keys = keys_;
  args.values = values_;
  args.mask = static_cast<std::uint32_t>(table_capacity_ - 1);
  args.key_epoch = epoch_;
  args.value_epoch = 0xFFFFFFFFULL - epoch_;
  args.overflow = overflow_;

  g_launches.fetch_add(1, std::memory_order_relaxed);
  context_->queue()
      .parallel_for<PreprocessKernel>(
          sycl::nd_range<1>{width, 1},
          [args](sycl::nd_item<1> item) {
            const std::size_t begin =
                item.get_group_linear_id() * args.blocks_per_group;
            const std::size_t end =
                std::min(args.blocks, begin + args.blocks_per_group);
            for (std::size_t block = begin; block < end; ++block) {
              processBlock(args, block);
            }
          })
      .wait_and_throw();

  if (*overflow_ != 0) {
    return false;
  }
  // VoxelFilter's selection, in index order.
  const bool intensity_test = min_intensity > 0.0F;
  const std::uint64_t mine = (0xFFFFFFFFULL - epoch_) << 32;
  out.reserve(count / 4);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint32_t slot = slots_[i];
    if (slot == kNoSlot || values_[slot] != (mine | i)) {
      continue;
    }
    const Point &point = reinterpret_cast<const Point *>(output_)[i];
    if (intensity_test && !(point.intensity >= min_intensity)) {
      continue;
    }
    out.push_back(point);
  }
  return true;
}

} // namespace mslam
