#pragma once

#include "common/Points.hh"

#include <filesystem>

namespace mslam {

PointCloud readPlyPointCloud(const std::filesystem::path &path);
void writePlyPointCloudBinary(const std::filesystem::path &path,
                              const PointCloud &cloud);

} // namespace mslam
