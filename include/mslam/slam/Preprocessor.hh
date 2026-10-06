#pragma once

#include "msensor/interface/IImu.hh"
#include "mslam/common/Points.hh"
#include "mslam/config/IConfig.hh"

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <spdlog/spdlog.h>
#include <vector>

namespace mslam {

std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter);

std::shared_ptr<Scan> removePointsNearCenter(const Scan &input,
                                             float min_distance);

std::shared_ptr<Scan> filterByIntensity(const Scan &input, float min_intensity);

/// Encapsulates the full preprocessing pipeline configured once at
/// construction. Avoids re-reading config fields on every scan iteration.
class Preprocessor {
public:
  Preprocessor(const PreProcessor &config,
               std::shared_ptr<spdlog::logger> logger);

  /// Range filter only — used during map initialisation.
  std::shared_ptr<Scan> filterNearCenter(const Scan &scan) const;

  /// Full pipeline: optional IMU deskew → range filter → downsample → intensity
  /// filter. The IMU samples are filtered to the scan window inside deskew().
  std::shared_ptr<Scan>
  process(const Scan &scan,
          const std::vector<msensor::IMUData> &imu_samples) const;

private:
  PreProcessor config_;
  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace mslam