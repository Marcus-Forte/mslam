#pragma once

#include "common/Points.hh"

#include <filesystem>

namespace mslam {

PointCloud readPlyPointCloud(const std::filesystem::path &path);

} // namespace mslam
