#pragma once

#include "mslam/config/IConfig.hh"
#include "mslam/map/VoxelHashMap.hh"

#include <memory>

namespace mslam {

/// Builds the registration map described by `parameters` (resolution, points
/// per voxel and search radius). The radius is applied before the map is
/// returned, so Slam sees it when it checks the correspondence gate.
/// @p backend Host: a VoxelHashMap, searched by the registrations on the host;
/// Sycl: a DeviceVoxelHashMap on executionContextFor(Sycl), searched in kernels.
std::shared_ptr<VoxelHashMap>
makeVoxelHashMap(const MapParameters &parameters,
                 ComputeBackend backend = ComputeBackend::Host);

/// The map of @p backend without a search radius set (ScanSequenceRegistration).
std::unique_ptr<VoxelHashMap>
makeVoxelHashMap(float resolution, std::size_t max_points_per_voxel,
                 ComputeBackend backend);

} // namespace mslam
