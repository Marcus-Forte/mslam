#include "map/VoxelHashMap.hh"
#include <gtest/gtest.h>

using namespace mslam;

class TestVoxelHashMap : public ::testing::Test {
public:
  void SetUp() override { map_ = std::make_unique<VoxelHashMap>(0.1, 5); }

protected:
  std::unique_ptr<VoxelHashMap> map_;
};

TEST_F(TestVoxelHashMap, test_max_points_per_bucket) {
  for (int i = 0; i < 10; ++i) {
    map_->addScan({{0.5, 0.5}});
  }
  auto map_rep = map_->getPointCloudRepresentation();
  EXPECT_EQ(map_rep.size(), 5);

  EXPECT_EQ(map_rep[0][0], 0.5);
  EXPECT_EQ(map_rep[0][1], 0.5);

  EXPECT_EQ(map_rep[4][0], 0.5);
  EXPECT_EQ(map_rep[4][1], 0.5);
}

TEST_F(TestVoxelHashMap, query_point_inside_voxel_corners) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  map_->addScan({{0.9, 0.9}, {0.1, 0.1}, {0.1, 0.9}, {0.9, 0.1}});

  auto neighbor = map_->getClosestNeighbor({0.8, 0.8});
  // dist (0.8, 0.8) -> (0.9 ,0.9) = norm(0.1, 0.1)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.1, 0.1).norm(), 1e-5);
  EXPECT_EQ(neighbor.first[0], 0.9);
  EXPECT_EQ(neighbor.first[1], 0.9);

  neighbor = map_->getClosestNeighbor({0.3, 0.3});
  // dist (0.3, 0.3) -> (0.1, 0.1) = norm(0.2, 0.2)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.2, 0.2).norm(), 1e-5);
  EXPECT_EQ(neighbor.first[0], 0.1);
  EXPECT_EQ(neighbor.first[1], 0.1);

  neighbor = map_->getClosestNeighbor({0.15, 0.75});
  // dist (0.15, 0.75) -> (0.1 0.9) = norm(0.05, 0.15)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.05, 0.15).norm(), 1e-5);
  EXPECT_EQ(neighbor.first[0], 0.1);
  EXPECT_EQ(neighbor.first[1], 0.9);

  neighbor = map_->getClosestNeighbor({0.7, 0.2});
  // dist (0.7, 0.2) -> (0.9, 0.1) = norm(0.2, 0.1)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.2, 0.1).norm(), 1e-5);
  EXPECT_EQ(neighbor.first[0], 0.9);
  EXPECT_EQ(neighbor.first[1], 0.1);
}

TEST_F(TestVoxelHashMap, query_point_outside_voxel_corners) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  map_->addScan({{0.9, 0.9}, {0.1, 0.1}, {0.1, 0.9}, {0.9, 0.1}});

  auto neighbor = map_->getClosestNeighbor({-0.5, -0.5});
  // dist (-0.5, -0.5) -> (0.1 ,0.1) = norm(0.6, 0.6)
  EXPECT_NEAR(neighbor.second, Eigen::Vector2d(0.6, 0.6).norm(), 1e-5);
  EXPECT_EQ(neighbor.first[0], 0.1);
  EXPECT_EQ(neighbor.first[1], 0.1);

  neighbor = map_->getClosestNeighbor({-1.5, -1.5});
  // Point outside voxel resolution, return convention is (0,0) and distance is
  // numeric max.
  EXPECT_EQ(neighbor.second, std::numeric_limits<double>::max());
  EXPECT_EQ(neighbor.first[0], 0.0);
  EXPECT_EQ(neighbor.first[1], 0.0);
}

TEST_F(TestVoxelHashMap, query_point_adjacent_voxels) {
  map_ = std::make_unique<VoxelHashMap>(1.0, 5);
  map_->setNumAdjacentVoxelSearch(1);
  map_->addScan({{0.5, 0.5}});

  auto neighbor = map_->getClosestNeighbor({1.5, 1.5});
  // query inside default adjacent voxel.
  EXPECT_EQ(neighbor.first[0], 0.5);
  EXPECT_EQ(neighbor.first[1], 0.5);

  neighbor = map_->getClosestNeighbor({2.5, 2.5});
  // query outside default adjacent voxel.
  EXPECT_EQ(neighbor.first[0], 0.0);
  EXPECT_EQ(neighbor.first[1], 0.0);
  // Increase adjacent search. Hereafter 0.5 will be found.
  map_->setNumAdjacentVoxelSearch(2);
  neighbor = map_->getClosestNeighbor({2.5, 2.5});
  EXPECT_EQ(neighbor.first[0], 0.5);
  EXPECT_EQ(neighbor.first[1], 0.5);
}
