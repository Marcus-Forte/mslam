#pragma once

#include "common/Points.hh"
#include "config/IConfig.hh"

#include <Eigen/Dense>
#include <cstdint>

namespace mslam {

std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter);

std::shared_ptr<Scan> removePointsNearCenter(const Scan &input,
                                             float min_distance);

std::shared_ptr<Scan> filterByIntensity(const Scan &input, float min_intensity);

std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion);

/// Time-aware deskew: scales the twist from relative_motion (observed over
/// delta_t seconds) to match the actual scan duration derived from scan_rate.
std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion,
                             unsigned int scan_rate, double delta_t);

/// Encapsulates the full preprocessing pipeline configured once at
/// construction. Avoids re-reading config fields on every scan iteration.
class Preprocessor {
public:
  explicit Preprocessor(const PreProcessor &config);

  /// Range filter only — used during map initialisation.
  std::shared_ptr<Scan> filterNearCenter(const Scan &scan) const;

  /// Full pipeline: optional deskew → range filter → downsample → intensity
  /// filter. Pass last_scan_timestamp_ns == 0 to skip time-aware deskew.
  std::shared_ptr<Scan> process(const Scan &scan,
                                const Eigen::Affine3d &last_delta,
                                uint64_t last_scan_timestamp_ns) const;

private:
  PreProcessor config_;
};

} // namespace mslam