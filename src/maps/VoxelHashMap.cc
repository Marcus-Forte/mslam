#include "map/VoxelHashMap.hh"

namespace mslam {

static const std::array<Voxel3, 27> voxel_shifts{
    {Voxel3{0, 0, 0},   Voxel3{1, 0, 0},   Voxel3{-1, 0, 0},  Voxel3{0, 1, 0},
     Voxel3{0, -1, 0},  Voxel3{0, 0, 1},   Voxel3{0, 0, -1},  Voxel3{1, 1, 0},
     Voxel3{1, -1, 0},  Voxel3{-1, 1, 0},  Voxel3{-1, -1, 0}, Voxel3{1, 0, 1},
     Voxel3{1, 0, -1},  Voxel3{-1, 0, 1},  Voxel3{-1, 0, -1}, Voxel3{0, 1, 1},
     Voxel3{0, 1, -1},  Voxel3{0, -1, 1},  Voxel3{0, -1, -1}, Voxel3{1, 1, 1},
     Voxel3{1, 1, -1},  Voxel3{1, -1, 1},  Voxel3{1, -1, -1}, Voxel3{-1, 1, 1},
     Voxel3{-1, 1, -1}, Voxel3{-1, -1, 1}, Voxel3{-1, -1, -1}}};

VoxelHashMap::VoxelHashMap(float voxel_size, size_t max_points_per_voxel)
    : voxel_size_(voxel_size), max_points_per_voxel_(max_points_per_voxel),
      adjacent_voxels_(1) {}

void VoxelHashMap::addScan(const PointCloud3 &scan) {

  for (const auto &point : scan.points) {

    const auto voxel = PointToVoxel({point.x, point.y, point.z}, voxel_size_);

    auto search = map_.find(voxel);
    if (search != map_.end()) {
      auto &bucket = search->second;
      if (bucket.points.size() != max_points_per_voxel_) {
        bucket.points.emplace_back(point);
      }
    } else { // new bucket
      PointCloud3 new_bucket;
      new_bucket.points.reserve(max_points_per_voxel_);
      new_bucket.points.emplace_back(point);
      map_.insert({voxel, std::move(new_bucket)});
    }
  }
}

IMap::Neighbor VoxelHashMap::getClosestNeighbor(const Point3 &query) const {

  const auto &voxel = PointToVoxel(query, voxel_size_);

  const Eigen::Vector3f query_eigen{query.x, query.y, query.z};

  Point3 closest_neighbor = {0, 0, 0};
  double closest_distance = std::numeric_limits<double>::max();
  // Search each neighbor
  std::for_each(
      voxel_shifts.begin(), voxel_shifts.end(), [&](const auto &voxel_shift) {
        const auto query_voxel = voxel + voxel_shift;
        auto search = map_.find(query_voxel);
        if (search != map_.end()) {
          const auto &bucket_points = search->second;
          const auto &neighbor = *std::min_element(
              bucket_points.points.cbegin(), bucket_points.points.cend(),
              [&](const auto &lhs, const auto &rhs) {
                /// \todo optimize
                const Eigen::Vector3f &lhs_vec = lhs.getVector3fMap();
                const Eigen::Vector3f &rhs_vec = rhs.getVector3fMap();

                return (lhs_vec - query_eigen).norm() <
                       (rhs_vec - query_eigen).norm();
              });

          double distance = (neighbor.getVector3fMap() - query_eigen).norm();
          if (distance < closest_distance) {
            closest_neighbor = neighbor;
            closest_distance = distance;
          }
        }
      });
  return {closest_neighbor, closest_distance};
}

/**
 * @brief Get a Point Cloud Representation. Copy might be made.
 *
 * @return PointCloud2D
 */
const PointCloud3 &VoxelHashMap::getPointCloudRepresentation() const {
  map_rep_.points.clear();
  map_rep_.points.reserve(map_.size() *
                          static_cast<size_t>(max_points_per_voxel_));
  std::for_each(map_.cbegin(), map_.cend(), [&](const auto &map_element) {
    const auto &voxel_buckets = map_element.second;
    map_rep_.points.insert(map_rep_.points.begin(),
                           voxel_buckets.points.cbegin(),
                           voxel_buckets.points.cend());
  });
  // map_rep_.shrink_to_fit();
  return map_rep_;
}

/**
 * @brief Set number of adjacent voxels from the query point to search for
 * buckets. This number exponentially decreases performance, albeit to as much
 * for low numbers.
 * @param adjacent_voxels
 */
void VoxelHashMap::setNumAdjacentVoxelSearch(int adjacent_voxels) {
  adjacent_voxels_ = adjacent_voxels;
}

} // namespace mslam
