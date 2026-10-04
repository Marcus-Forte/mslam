#include "map/VoxelHashMap.hh"
#include <gtest/gtest.h>
#include <random>

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
