#pragma once

#include "mslam/common/Points.hh"
#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mslam {

using Voxel3 = Eigen::Vector3i;

inline Voxel3 PointToVoxel(const Point &point, float inverse_voxel_size) {
  return {static_cast<int>(std::floor(point.x * inverse_voxel_size)),
          static_cast<int>(std::floor(point.y * inverse_voxel_size)),
          static_cast<int>(std::floor(point.z * inverse_voxel_size))};
}

} // namespace mslam

/// Hash for voxel / integer lattice keys. Defined inline so lookups do not pay
/// a call per hash. tsl::robin_map masks the low bits of the hash, so the
/// result goes through a 64-bit finalizer that spreads every input bit down.
template <> struct std::hash<mslam::Voxel3> {
  std::size_t operator()(const mslam::Voxel3 &voxel) const noexcept {
    const std::uint64_t xy =
        static_cast<std::uint32_t>(voxel.x()) |
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(voxel.y()))
         << 32);
    std::uint64_t h = xy * 0x9E3779B97F4A7C15ULL;
    h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(voxel.z())) *
         0xC2B2AE3D27D4EB4FULL;
    h ^= h >> 32;
    h *= 0xD6E8FEB86659FD93ULL;
    h ^= h >> 32;
    return static_cast<std::size_t>(h);
  }
};
