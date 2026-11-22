#include "mslam/slam/MapFactory.hh"

#include "mslam/map/SyclVoxelHashMap.hh"

#include <cstdlib>
#include <cstring>

namespace mslam {

// The config validates against one constant and the map against the other;
// they must accept the same radii.
static_assert(MapParameters::kMaxSearchRadius ==
                  VoxelHashMap::kMaxAdjacentVoxels,
              "map.search_radius must accept exactly the radii the map does");

std::shared_ptr<VoxelHashMap>
makeVoxelHashMap(const MapParameters &parameters) {
  // The SYCL map: the registrations search its device mirror in kernels.
  // MSLAM_HOST_SEARCH=1 (comparisons only) builds the plain host map instead.
  const char *host_search = std::getenv("MSLAM_HOST_SEARCH");
  const bool host = host_search != nullptr && std::strcmp(host_search, "1") == 0;
  std::shared_ptr<VoxelHashMap> map =
      host ? std::make_shared<VoxelHashMap>(parameters.resolution,
                                            parameters.max_points_per_voxel)
           : std::make_shared<SyclVoxelHashMap>(
                 parameters.resolution, parameters.max_points_per_voxel);
  map->setNumAdjacentVoxelSearch(parameters.search_radius);
  return map;
}

} // namespace mslam
