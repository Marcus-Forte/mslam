#include "mslam/map/VoxelHashMap.hh"
#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>

using namespace mslam;

class TestVoxelHashMap : public ::testing::Test {
public:
  void SetUp() override { map_ = std::make_unique<VoxelHashMap>(0.1, 5); }

protected:
  std::unique_ptr<VoxelHashMap> map_;
};

TEST_F(TestVoxelHashMap, test_max_points_per_bucket) {

  for (int i = 0; i < 10; ++i) {
    PointCloud scan;
    scan.emplace_back(0.5, 0.5, 0.5);
    map_->addScan(scan);
  }
  auto map_rep = map_->getPointCloudRepresentation();
  EXPECT_EQ(map_rep.size(), 5);

  EXPECT_EQ(map_rep[0].x, 0.5);
  EXPECT_EQ(map_rep[0].y, 0.5);
  EXPECT_EQ(map_rep[0].z, 0.5);

  EXPECT_EQ(map_rep[4].x, 0.5);
  EXPECT_EQ(map_rep[4].y, 0.5);
  EXPECT_EQ(map_rep[4].z, 0.5);
}

TEST_F(TestVoxelHashMap, query_point_inside_voxel_corners) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  PointCloud scan;
  scan.emplace_back(0.9, 0.9, 0.0);
  scan.emplace_back(0.1, 0.1, 0.0);
  scan.emplace_back(0.1, 0.9, 0.0);
  scan.emplace_back(0.9, 0.1, 0.0);

  map_->addScan(scan);

  auto neighbor = map_->getClosestNeighbor({0.8, 0.8, 0.0});
  // squared dist (0.8, 0.8) -> (0.9 ,0.9) = squaredNorm(0.1, 0.1)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.1, 0.1).squaredNorm(), 1e-5);
  EXPECT_NEAR(neighbor.first.x, 0.9, 1e-5);
  EXPECT_NEAR(neighbor.first.y, 0.9, 1e-5);
  EXPECT_NEAR(neighbor.first.z, 0.0, 1e-5);

  neighbor = map_->getClosestNeighbor({0.3, 0.3, 0.0});
  // squared dist (0.3, 0.3) -> (0.1, 0.1) = squaredNorm(0.2, 0.2)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.2, 0.2).squaredNorm(), 1e-5);
  EXPECT_NEAR(neighbor.first.x, 0.1, 1e-5);
  EXPECT_NEAR(neighbor.first.y, 0.1, 1e-5);

  neighbor = map_->getClosestNeighbor({0.15, 0.75, 0.0});
  // squared dist (0.15, 0.75) -> (0.1 0.9) = squaredNorm(0.05, 0.15)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.05, 0.15).squaredNorm(), 1e-5);
  EXPECT_NEAR(neighbor.first.x, 0.1, 1e-5);
  EXPECT_NEAR(neighbor.first.y, 0.9, 1e-5);

  neighbor = map_->getClosestNeighbor({0.7, 0.2, 0.0});
  // squared dist (0.7, 0.2) -> (0.9, 0.1) = squaredNorm(0.2, 0.1)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.2, 0.1).squaredNorm(), 1e-5);
  EXPECT_NEAR(neighbor.first.x, 0.9, 1e-5);
  EXPECT_NEAR(neighbor.first.y, 0.1, 1e-5);
}

TEST_F(TestVoxelHashMap, query_point_outside_voxel_corners) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);

  PointCloud scan;
  scan.emplace_back(0.9, 0.9, 0.0);
  scan.emplace_back(0.1, 0.1, 0.0);
  scan.emplace_back(0.1, 0.9, 0.0);
  scan.emplace_back(0.9, 0.1, 0.0);

  map_->addScan(scan);

  auto neighbor = map_->getClosestNeighbor({-0.5, -0.5, 0.0});
  // squared dist (-0.5, -0.5) -> (0.1 ,0.1) = squaredNorm(0.6, 0.6)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.6, 0.6).squaredNorm(), 1e-5);
  EXPECT_NEAR(neighbor.first.x, 0.1, 1e-5);
  EXPECT_NEAR(neighbor.first.y, 0.1, 1e-5);

  neighbor = map_->getClosestNeighbor({-1.5, -1.5, 0.0});
  // Point outside voxel resolution, return convention is (0,0) and distance
  // is numeric max.
  EXPECT_EQ(neighbor.second, std::numeric_limits<float>::max());
  EXPECT_EQ(neighbor.first.x, 0.0);
  EXPECT_EQ(neighbor.first.y, 0.0);
}

TEST_F(TestVoxelHashMap, query_point_adjacent_voxels) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);

  PointCloud scan;
  scan.emplace_back(0.5, 0.5, 0.0);

  map_->setNumAdjacentVoxelSearch(1);
  map_->addScan(scan);

  auto neighbor = map_->getClosestNeighbor({1.5, 1.5, 0.0});
  // query inside default adjacent voxel.
  EXPECT_EQ(neighbor.first.x, 0.5);
  EXPECT_EQ(neighbor.first.y, 0.5);

  neighbor = map_->getClosestNeighbor({2.5, 2.5, 0.0});
  // query outside default adjacent voxel.
  EXPECT_EQ(neighbor.first.x, 0.0);
  EXPECT_EQ(neighbor.first.y, 0.0);
  // Increase adjacent search. Hereafter 0.5 will be found.
  map_->setNumAdjacentVoxelSearch(2);
  neighbor = map_->getClosestNeighbor({2.5, 2.5, 0.0});
  EXPECT_EQ(neighbor.first.x, 0.5);
  EXPECT_EQ(neighbor.first.y, 0.5);
}

TEST_F(TestVoxelHashMap, query_multiple_closest_neighbors) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);

  PointCloud scan;
  scan.emplace_back(0.1, 0.1, 0.0);
  scan.emplace_back(0.2, 0.2, 0.0);
  scan.emplace_back(0.8, 0.8, 0.0);
  scan.emplace_back(1.1, 1.1, 0.0);

  map_->setNumAdjacentVoxelSearch(1);
  map_->addScan(scan);

  const auto neighbors = map_->getClosestNNeighbors({0.0, 0.0, 0.0}, 3);
  ASSERT_EQ(neighbors.size(), 3);

  EXPECT_NEAR(neighbors[0].first.x, 0.1, 1e-5);
  EXPECT_NEAR(neighbors[0].first.y, 0.1, 1e-5);
  EXPECT_NEAR(neighbors[1].first.x, 0.2, 1e-5);
  EXPECT_NEAR(neighbors[1].first.y, 0.2, 1e-5);
  EXPECT_NEAR(neighbors[2].first.x, 0.8, 1e-5);
  EXPECT_NEAR(neighbors[2].first.y, 0.8, 1e-5);

  EXPECT_LE(neighbors[0].second, neighbors[1].second);
  EXPECT_LE(neighbors[1].second, neighbors[2].second);
}

TEST_F(TestVoxelHashMap, query_multiple_neighbors_invalid_count) {
  const auto neighbors = map_->getClosestNNeighbors({0.0, 0.0, 0.0}, 0);
  EXPECT_TRUE(neighbors.empty());
}

TEST_F(TestVoxelHashMap, prune_removes_voxels_beyond_range) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  PointCloud scan;
  scan.emplace_back(0.5, 0.5, 0.5);
  scan.emplace_back(10.5, 10.5, 10.5);
  map_->addScan(scan);

  map_->prune(Point{0.0F, 0.0F, 0.0F}, 2.0F, 0);
  EXPECT_EQ(map_->size(), 1U);

  const auto representation = map_->getPointCloudRepresentation();
  ASSERT_EQ(representation.size(), 1U);
  EXPECT_FLOAT_EQ(representation[0].x, 0.5F);
}

TEST_F(TestVoxelHashMap, prune_budget_keeps_voxels_nearest_to_center) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  for (int i = 0; i < 10; ++i) {
    PointCloud scan;
    scan.emplace_back(static_cast<float>(i) + 0.5F, 0.5F, 0.5F);
    map_->addScan(scan);
  }
  ASSERT_EQ(map_->size(), 10U);

  map_->prune(Point{0.5F, 0.5F, 0.5F}, 0.0F, 4);
  EXPECT_LE(map_->size(), 4U);

  // Only the voxels closest to the center survive.
  for (const auto &point : map_->getPointCloudRepresentation()) {
    EXPECT_LE(point.x, 4.5F);
  }
}

TEST_F(TestVoxelHashMap, prune_bounds_unbounded_insertion) {
  map_ = std::make_unique<VoxelHashMap>(0.1, 5);
  const Point center{0.0F, 0.0F, 0.0F};

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> noise(-5.0F, 5.0F);

  for (int scan = 0; scan < 200; ++scan) {
    PointCloud cloud;
    cloud.reserve(500);
    for (int i = 0; i < 500; ++i) {
      cloud.emplace_back(noise(rng), noise(rng), noise(rng));
    }
    map_->addScan(cloud);
    map_->prune(center, 0.0F, 1000);
  }

  EXPECT_LE(map_->size(), 1000U);
}

TEST_F(TestVoxelHashMap, representation_is_rebuilt_after_prune) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  PointCloud scan;
  scan.emplace_back(0.5, 0.5, 0.5);
  scan.emplace_back(10.5, 10.5, 10.5);
  map_->addScan(scan);
  ASSERT_EQ(map_->getPointCloudRepresentation().size(), 2U);

  map_->prune(Point{0.0F, 0.0F, 0.0F}, 2.0F, 0);
  EXPECT_EQ(map_->getPointCloudRepresentation().size(), 1U);
}

TEST_F(TestVoxelHashMap, reuses_storage_after_pruning_voxels) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 2);
  PointCloud scan;
  scan.emplace_back(0.5F, 0.5F, 0.5F);
  scan.emplace_back(10.5F, 10.5F, 10.5F);
  map_->addScan(scan);

  map_->prune(Point{0.5F, 0.5F, 0.5F}, 0.0F, 1);

  PointCloud next_scan;
  next_scan.emplace_back(20.5F, 20.5F, 20.5F);
  map_->addScan(next_scan);

  ASSERT_EQ(map_->getPointCloudRepresentation().size(), 2U);
  const auto neighbor = map_->getClosestNeighbor({20.5F, 20.5F, 20.5F});
  EXPECT_FLOAT_EQ(neighbor.first.x, 20.5F);
  EXPECT_FLOAT_EQ(neighbor.first.y, 20.5F);
  EXPECT_FLOAT_EQ(neighbor.first.z, 20.5F);
}

namespace {

// Reference implementation: scan every stored point whose voxel lies within
// `radius` voxels of the query voxel, in the same shift order as the map.
struct ReferenceSearch {
  const PointCloud &points;
  float inverse_voxel_size;
  int radius;

  bool inNeighbourhood(const Point &point, const Voxel3 &query_voxel) const {
    const Voxel3 voxel = PointToVoxel(point, inverse_voxel_size);
    return (voxel - query_voxel).cwiseAbs().maxCoeff() <= radius;
  }

  std::vector<IMap::Neighbor> sortedCandidates(const Point &query) const {
    const Voxel3 query_voxel = PointToVoxel(query, inverse_voxel_size);
    std::vector<IMap::Neighbor> candidates;
    for (const auto &point : points) {
      if (!inNeighbourhood(point, query_voxel)) {
        continue;
      }
      const float dx = point.x - query.x;
      const float dy = point.y - query.y;
      const float dz = point.z - query.z;
      candidates.emplace_back(Point{point.x, point.y, point.z},
                              dx * dx + dy * dy + dz * dz);
    }
    std::stable_sort(
        candidates.begin(), candidates.end(),
        [](const auto &a, const auto &b) { return a.second < b.second; });
    return candidates;
  }
};

PointCloud randomCloud(std::mt19937 &rng, int count, float extent,
                       bool snap_to_grid) {
  std::uniform_real_distribution<float> coordinate(-extent, extent);
  PointCloud cloud;
  for (int i = 0; i < count; ++i) {
    Point point(coordinate(rng), coordinate(rng), coordinate(rng));
    if (snap_to_grid) {
      point.x = std::round(point.x * 20.0F) / 20.0F;
      point.y = std::round(point.y * 20.0F) / 20.0F;
      point.z = std::round(point.z * 20.0F) / 20.0F;
    }
    cloud.push_back(point);
  }
  return cloud;
}

} // namespace

TEST(VoxelHashMapSearch, ClosestNeighborMatchesExhaustiveSearch) {
  std::mt19937 rng(1234);
  for (const int radius : {1, 2}) {
    for (const bool snapped : {false, true}) {
      VoxelHashMap map(0.1F, 5);
      map.setNumAdjacentVoxelSearch(radius);
      // Several scans, so buckets fill up and overflowing points are dropped.
      for (int scan = 0; scan < 4; ++scan) {
        map.addScan(randomCloud(rng, 4000, 1.0F, snapped));
      }
      const PointCloud stored = map.getPointCloudRepresentation();
      const ReferenceSearch reference{stored, 10.0F, radius};

      for (const auto &query : randomCloud(rng, 3000, 1.3F, false)) {
        const auto candidates = reference.sortedCandidates(query);
        const auto result = map.getClosestNeighbor(query);
        if (candidates.empty()) {
          EXPECT_EQ(result.second, std::numeric_limits<float>::max());
          continue;
        }
        // Distances must be identical. Only an exact tie may pick a different
        // point, and the lowest-shift rule is checked below by value.
        ASSERT_EQ(result.second, candidates.front().second);
      }
    }
  }
}

TEST(VoxelHashMapSearch, ClosestNNeighborsMatchExhaustiveSearch) {
  std::mt19937 rng(99);
  for (const bool snapped : {false, true}) {
    VoxelHashMap map(0.1F, 5);
    for (int scan = 0; scan < 4; ++scan) {
      map.addScan(randomCloud(rng, 4000, 1.0F, snapped));
    }
    const PointCloud stored = map.getPointCloudRepresentation();
    const ReferenceSearch reference{stored, 10.0F, 1};

    for (const int count : {1, 3, 5, 12}) {
      for (const auto &query : randomCloud(rng, 1500, 1.3F, false)) {
        const auto candidates = reference.sortedCandidates(query);
        const auto result = map.getClosestNNeighbors(query, count);
        const auto expected_size =
            std::min<size_t>(candidates.size(), static_cast<size_t>(count));
        ASSERT_EQ(result.size(), expected_size);
        for (size_t i = 0; i < expected_size; ++i) {
          // The k-th distance is unique even when equidistant points could be
          // returned in either order.
          ASSERT_EQ(result[i].second, candidates[i].second) << "rank " << i;
        }
      }
    }
  }
}

TEST(VoxelHashMapSearch, FindClosestNeighborsFillsCallerBuffer) {
  VoxelHashMap map(1.0F, 5);
  PointCloud scan;
  scan.emplace_back(0.1F, 0.1F, 0.0F);
  scan.emplace_back(0.2F, 0.2F, 0.0F);
  scan.emplace_back(0.8F, 0.8F, 0.0F);
  map.addScan(scan);

  std::array<IMap::Neighbor, 8> buffer;
  const auto count =
      map.findClosestNeighbors({0.0F, 0.0F, 0.0F}, std::span(buffer.data(), 2));
  ASSERT_EQ(count, 2U);
  EXPECT_FLOAT_EQ(buffer[0].first.x, 0.1F);
  EXPECT_FLOAT_EQ(buffer[1].first.x, 0.2F);
  // Entries past the requested size are untouched.
  EXPECT_EQ(buffer[2].second, 0.0F);

  EXPECT_EQ(
      map.findClosestNeighbors({0.0F, 0.0F, 0.0F}, std::span<IMap::Neighbor>{}),
      0U);
  // 5 neighbors requested, only 3 exist.
  EXPECT_EQ(
      map.findClosestNeighbors({0.0F, 0.0F, 0.0F}, std::span(buffer.data(), 5)),
      3U);
}

TEST(VoxelHashMapSearch, LargeCoordinatesAreNotSkippedByVoxelBounds) {
  // Far from the origin float rounding of voxel boundaries is coarse; the
  // voxel lower bound must stay conservative.
  std::mt19937 rng(5);
  VoxelHashMap map(0.1F, 5);
  std::uniform_real_distribution<float> offset(-0.5F, 0.5F);
  PointCloud cloud;
  for (int i = 0; i < 5000; ++i) {
    cloud.emplace_back(900.0F + offset(rng), -700.0F + offset(rng),
                       300.0F + offset(rng));
  }
  map.addScan(cloud);
  const PointCloud stored = map.getPointCloudRepresentation();
  const ReferenceSearch reference{stored, 10.0F, 1};
  for (int i = 0; i < 3000; ++i) {
    const Point query(900.0F + offset(rng), -700.0F + offset(rng),
                      300.0F + offset(rng));
    const auto candidates = reference.sortedCandidates(query);
    const auto result = map.getClosestNeighbor(query);
    if (!candidates.empty()) {
      ASSERT_EQ(result.second, candidates.front().second);
    }
  }
}

TEST(VoxelHashMapSearch, SearchRadiusIsValidated) {
  VoxelHashMap map(0.1F, 5);
  EXPECT_THROW(map.setNumAdjacentVoxelSearch(-1), std::invalid_argument);
  EXPECT_THROW(
      map.setNumAdjacentVoxelSearch(VoxelHashMap::kMaxAdjacentVoxels + 1),
      std::invalid_argument);
  EXPECT_NO_THROW(
      map.setNumAdjacentVoxelSearch(VoxelHashMap::kMaxAdjacentVoxels));
}

TEST(VoxelHashMapSearch, VoxelHashSpreadsLowBits) {
  // robin_map masks the low bits: a lattice of voxels must not collapse onto
  // few buckets.
  std::hash<Voxel3> hash;
  std::set<std::size_t> buckets;
  for (int x = 0; x < 32; ++x)
    for (int y = 0; y < 32; ++y)
      for (int z = 0; z < 8; ++z)
        buckets.insert(hash(Voxel3(x, y, z)) & 0x3FFF);
  // 8192 keys into 16384 buckets: a random hash occupies ~6.4k of them.
  EXPECT_GT(buckets.size(), 5500U);
}

TEST(VoxelHashMapSearch, ClosestNeighborTieResolvesToLowestShift) {
  // Equidistant points in the home voxel and in the dx = -1 voxel. An
  // exhaustive scan in shift order returns the dx = -1 point even though the
  // home voxel is visited first.
  VoxelHashMap map(1.0F, 5);
  PointCloud scan;
  scan.emplace_back(0.75F, 0.5F, 0.5F);  // home voxel (0,0,0)
  scan.emplace_back(-0.25F, 0.5F, 0.5F); // voxel (-1,0,0)
  map.addScan(scan);

  const auto neighbor = map.getClosestNeighbor({0.25F, 0.5F, 0.5F});
  EXPECT_FLOAT_EQ(neighbor.second, 0.25F);
  EXPECT_FLOAT_EQ(neighbor.first.x, -0.25F);
}
