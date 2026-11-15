#pragma once

#include "mslam/common/Points.hh"

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <span>

namespace mslam {

PointCloud readPlyPointCloud(const std::filesystem::path &path);

/// Size in bytes of the binary PLY `writePlyPointCloud` writes for
/// `num_points` points.
std::uintmax_t plyFileSize(std::size_t num_points);

/// Writes the concatenation of `chunks` as a binary little-endian PLY
/// (`property float x y z intensity`, the layout three.js' PLYLoader in
/// `viewer/` reads). The chunks are written in place, without being joined.
/// Returns the number of bytes written; throws std::runtime_error if the
/// stream fails.
std::uintmax_t writePlyPointCloud(std::ostream &out,
                                  std::span<const PointCloud> chunks);

/// Writes `cloud` to `path` (overwriting). Throws std::runtime_error if the
/// file cannot be opened or written.
std::uintmax_t writePlyPointCloud(const std::filesystem::path &path,
                                  const PointCloud &cloud);

} // namespace mslam
