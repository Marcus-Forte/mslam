#include "mslam/slam/MapFactory.hh"

#include "mslam/map/DeviceVoxelHashMap.hh"
#include "mslam/slam/ComputeContext.hh"

namespace mslam {

// The config validates against one constant and the map against the other;
// they must accept the same radii.
static_assert(MapParameters::kMaxSearchRadius ==
                  VoxelHashMap::kMaxAdjacentVoxels,
              "map.search_radius must accept exactly the radii the map does");

std::shared_ptr<VoxelHashMap>
makeVoxelHashMap(const MapParameters &parameters, ComputeBackend backend) {
  std::shared_ptr<VoxelHashMap> map = makeVoxelHashMap(
      parameters.resolution, parameters.max_points_per_voxel, backend);
  map->setNumAdjacentVoxelSearch(parameters.search_radius);
  return map;
}

std::unique_ptr<VoxelHashMap> makeVoxelHashMap(float resolution,
                                               std::size_t max_points_per_voxel,
                                               ComputeBackend backend) {
  // sycl: the registrations search the map's device mirror in kernels.
  if (backend == ComputeBackend::Sycl) {
    return std::make_unique<DeviceVoxelHashMap>(
        resolution, max_points_per_voxel, executionContextFor(backend));
  }
  return std::make_unique<VoxelHashMap>(resolution, max_points_per_voxel);
}

} // namespace mslam
