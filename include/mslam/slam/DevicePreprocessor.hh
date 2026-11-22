#pragma once

#include "msensor/interface/IImu.hh"
#include "mslam/common/Points.hh"
#include "mslam/slam/Deskew.hh"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace moptim {
class ExecutionContext;
}

namespace mslam {

/// Total kernel launches of all DevicePreprocessor objects (bench_slam).
std::uint64_t devicePreprocessLaunches();

/**
 * @brief Deskew, range filter and voxel-hash downsampling of a scan in one
 * kernel (Preprocessor::process with the voxel_hash filter on the sycl
 * backend), on a moptim execution context: a SYCL kernel, or host work-groups
 * on moptim's host backend.
 *
 * - Deskew: the scan is cut into blocks of kBlock points, whose boundaries do
 *   not depend on the number of work-groups. A block starts from the sensor
 *   orientation at its first point, from the IMU interval table
 *   (deskewIntervals()), and then advances point by point with deskewInto()'s
 *   zero-order-hold running product (the same memoized rotation increments).
 *   Block 0 is therefore deskewInto() bit for bit; later blocks differ from it
 *   in the last bits (they start from a closed form instead of the running
 *   product of all earlier points), whatever the width.
 * - Range test on the deskewed point, as VoxelFilter does.
 * - voxel_hash: each point that passes is inserted into a voxel table in USM
 *   whose entry keeps the smallest point index (an atomic minimum, so the result
 *   does not depend on the order the work-items run in). The host then keeps,
 *   in index order, the points that are their voxel's smallest index and pass
 *   the intensity test: VoxelFilter's "first point of a voxel represents it,
 *   even if the intensity test then rejects it".
 *
 * A scan whose points fall outside the table's 16-bit voxel coordinates is
 * filtered on the host (VoxelFilter on the deskewed points).
 */
class DevicePreprocessor {
public:
  static constexpr std::size_t kBlock = 256;

  /// @param context null: moptim's default context.
  explicit DevicePreprocessor(
      std::shared_ptr<moptim::ExecutionContext> context = nullptr);
  ~DevicePreprocessor();

  DevicePreprocessor(const DevicePreprocessor &) = delete;
  DevicePreprocessor &operator=(const DevicePreprocessor &) = delete;

  /// Work-groups of the kernel (at least 1).
  void setWidth(std::size_t width) { width_ = width == 0 ? 1 : width; }

  /// Deskews @p scan when @p delta_t > 0 and IMU samples cover it, then filters
  /// it into @p out. Returns false (out untouched) if the scan needs the host
  /// filter (voxel coordinates out of range); the deskewed points are then in
  /// deskewed().
  bool process(const Scan &scan,
               const std::vector<msensor::IMUData> &imu_samples, double delta_t,
               float min_distance, float voxel_size, float min_intensity,
               PointCloud &out);

  /// The deskewed points of the last process() (valid until the next one).
  const Point *deskewed() const;

  /// Whether the last process() deskewed the scan (IMU samples covered it).
  bool lastScanDeskewed() const { return last_deskewed_; }

private:
  void reserve(std::size_t points, std::size_t intervals);
  void nextEpoch();

  std::shared_ptr<moptim::ExecutionContext> context_;
  std::size_t width_ = 1;
  DeskewIntervals intervals_;

  std::size_t capacity_ = 0;          ///< Points.
  std::size_t interval_capacity_ = 0; ///< IMU intervals.
  std::size_t table_capacity_ = 0;    ///< Voxel table slots (a power of two).
  Point *input_ = nullptr;            ///< Copy of the scan (devices without host access).
  Point *output_ = nullptr;           ///< Deskewed points.
  std::uint32_t *slots_ = nullptr;    ///< Voxel table slot per point, or none.
  std::uint64_t *keys_ = nullptr;     ///< Epoch-tagged voxel keys.
  std::uint64_t *values_ = nullptr;   ///< Epoch-tagged smallest point index.
  double *boundaries_ = nullptr;
  double *omegas_ = nullptr;        ///< 3 per interval.
  double *orientations_ = nullptr;  ///< 9 per interval, column-major.
  std::uint32_t *overflow_ = nullptr;
  std::uint32_t epoch_ = 0;
  bool last_deskewed_ = false;
};

} // namespace mslam
