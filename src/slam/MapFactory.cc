#include "mslam/slam/MapFactory.hh"

namespace mslam {

// The config validates against one constant and the map against the other;
// they must accept the same radii.
static_assert(MapParameters::kMaxSearchRadius ==
                  VoxelHashMap::kMaxAdjacentVoxels,
              "map.search_radius must accept exactly the radii the map does");

std::shared_ptr<VoxelHashMap>
makeVoxelHashMap(const MapParameters &parameters) {
  auto map = std::make_shared<VoxelHashMap>(parameters.resolution,
                                            parameters.max_points_per_voxel);
  map->setNumAdjacentVoxelSearch(parameters.search_radius);
  return map;
}

} // namespace mslam
