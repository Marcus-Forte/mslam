#pragma once

#include "common/Points.hh"
#include "config/IConfig.hh"

#include <Eigen/Dense>

namespace mslam {

std::shared_ptr<Scan> downsample(const Scan &input, float voxel_size,
                                 DownsampleFilter filter);

std::shared_ptr<Scan> removePointsNearCenter(const Scan &input,
                                             float min_distance);

std::shared_ptr<Scan> filterByIntensity(const Scan &input, float min_intensity);

std::shared_ptr<Scan> deskew(const Scan &scan,
                             const Eigen::Affine3d &relative_motion);

} // namespace mslam