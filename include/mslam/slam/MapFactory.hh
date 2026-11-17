#pragma once

#include "mslam/config/IConfig.hh"
#include "mslam/map/VoxelHashMap.hh"

#include <memory>

namespace mslam {

/// Builds the registration map described by `parameters` (resolution, points
/// per voxel and search radius). The radius is applied before the map is
/// returned, so Slam sees it when it checks the correspondence gate.
std::shared_ptr<VoxelHashMap> makeVoxelHashMap(const MapParameters &parameters);

} // namespace mslam
