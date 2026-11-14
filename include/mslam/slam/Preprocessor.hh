#pragma once

#include "msensor/interface/IImu.hh"
#include "mslam/common/Points.hh"
#include "mslam/config/IConfig.hh"
#include "mslam/map/Voxel.hh"

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <spdlog/spdlog.h>
#include <tsl/robin_map.h>
#include <tsl/robin_set.h>
#include <vector>

namespace mslam {

/// Keeps the first point of every voxel, in the order the voxels are first
/// seen (VoxelHash) or the voxel centroids in that order (VoxelGrid).
/// \throws std::invalid_argument if voxel_size is not finite and positive.
std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter);

std::shared_ptr<Scan> removePointsNearCenter(const Scan &input,
                                             float min_distance);

/// Drops points below @p min_intensity. Always returns a new scan (a copy of
/// the input when the filter is disabled, min_intensity <= 0).
std::shared_ptr<Scan> filterByIntensity(const Scan &input, float min_intensity);

/// Range filter, voxel downsample and intensity filter fused in one pass.
///
/// Equivalent to removePointsNearCenter() -> downsample() ->
/// filterByIntensity() run in sequence, without the intermediate clouds. The
/// voxel tables are kept between calls, so a long-lived instance does not
/// allocate in steady state.
class VoxelFilter {
public:
  /// @param min_distance Points closer than this to the origin are dropped; a
  ///        negative value disables the range test.
  /// @param min_intensity Representatives below this are dropped; a value <= 0
  ///        disables the test. Applied to the voxel representative only, as in
  ///        the sequential pipeline.
  /// @param out Replaced by the result; its capacity is reused. Must not alias
  ///        @p in.
  /// \throws std::invalid_argument if voxel_size is not finite and positive.
  void apply(const PointCloud &in, float min_distance, float voxel_size,
             DownsampleFilter filter, float min_intensity, PointCloud &out);

private:
  struct PointAccumulator {
    double x = 0.0, y = 0.0, z = 0.0, intensity = 0.0;
    std::size_t count = 0;
  };

  tsl::robin_set<Voxel3> occupied_voxels_;
  tsl::robin_map<Voxel3, std::uint32_t> voxel_slots_;
  std::vector<PointAccumulator> accumulators_;
};

/// Encapsulates the full preprocessing pipeline configured once at
/// construction. Avoids re-reading config fields on every scan iteration and
/// keeps its working buffers between scans, so a scan costs no allocation once
/// the buffers have grown to the largest scan seen.
class Preprocessor {
public:
  Preprocessor(const PreProcessor &config,
               std::shared_ptr<spdlog::logger> logger);

  /// Range filter only — used during map initialisation.
  std::shared_ptr<Scan> filterNearCenter(const Scan &scan) const;

  /// Full pipeline: optional IMU deskew, then range filter, voxel downsample
  /// and intensity filter fused in one pass over the points. The IMU samples
  /// are filtered to the scan window inside deskew(). The result goes to
  /// @p out (header copied, points replaced, capacity reused).
  void process(const Scan &scan,
               const std::vector<msensor::IMUData> &imu_samples, Scan &out);

  /// Convenience overload allocating the result.
  std::shared_ptr<Scan>
  process(const Scan &scan, const std::vector<msensor::IMUData> &imu_samples);

private:
  PreProcessor config_;
  std::shared_ptr<spdlog::logger> logger_;

  // Working buffers reused across scans.
  PointCloud deskewed_;
  VoxelFilter voxel_filter_;
};

} // namespace mslam
