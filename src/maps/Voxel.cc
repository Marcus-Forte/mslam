#include "map/Voxel.hh"

std::size_t
std::hash<mslam::Voxel3>::operator()(const mslam::Voxel3 &voxel) const {
  const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
  return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
}
