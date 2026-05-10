#include "map/KDTreeMap.hh"
#include "map/OctreeMap.hh"
#include <gtest/gtest.h>

using namespace mslam;

TEST(TestKDTreeMapNeighbors, query_multiple_neighbors) {
  KDTreeMap map;

  PointCloud3 scan;
  scan.emplace_back(0.1F, 0.1F, 0.0F);
  scan.emplace_back(0.2F, 0.2F, 0.0F);
  scan.emplace_back(0.8F, 0.8F, 0.0F);
  scan.emplace_back(1.1F, 1.1F, 0.0F);
  map.addScan(scan);

  const auto neighbors = map.getClosestNNeighbors({0.0F, 0.0F, 0.0F}, 3);
  ASSERT_EQ(neighbors.size(), 3);

  EXPECT_NEAR(neighbors[0].first.x, 0.1F, 1e-5F);
  EXPECT_NEAR(neighbors[0].first.y, 0.1F, 1e-5F);
  EXPECT_NEAR(neighbors[1].first.x, 0.2F, 1e-5F);
  EXPECT_NEAR(neighbors[1].first.y, 0.2F, 1e-5F);
  EXPECT_NEAR(neighbors[2].first.x, 0.8F, 1e-5F);
  EXPECT_NEAR(neighbors[2].first.y, 0.8F, 1e-5F);

  EXPECT_LE(neighbors[0].second, neighbors[1].second);
  EXPECT_LE(neighbors[1].second, neighbors[2].second);
}

TEST(TestKDTreeMapNeighbors, invalid_neighbor_count) {
  KDTreeMap map;
  EXPECT_TRUE(map.getClosestNNeighbors({0.0F, 0.0F, 0.0F}, 0).empty());
}

TEST(TestKDTreeMapNeighbors, reports_zero_resolution) {
  KDTreeMap map;
  EXPECT_FLOAT_EQ(map.getResolution(), 0.0F);
}

TEST(TestOctreeMapNeighbors, stub_returns_single_neighbor) {
  OctreeMap map(1.0F);

  PointCloud3 scan;
  scan.emplace_back(0.1F, 0.1F, 0.0F);
  scan.emplace_back(0.8F, 0.8F, 0.0F);
  map.addScan(scan);

  const auto neighbors = map.getClosestNNeighbors({0.0F, 0.0F, 0.0F}, 3);
  ASSERT_EQ(neighbors.size(), 1);
  EXPECT_NEAR(neighbors[0].first.x, 0.1F, 1e-5F);
  EXPECT_NEAR(neighbors[0].first.y, 0.1F, 1e-5F);
}

TEST(TestOctreeMapNeighbors, invalid_neighbor_count) {
  OctreeMap map(1.0F);
  EXPECT_TRUE(map.getClosestNNeighbors({0.0F, 0.0F, 0.0F}, 0).empty());
}

TEST(TestOctreeMapNeighbors, reports_voxel_resolution) {
  OctreeMap map(1.0F);
  EXPECT_FLOAT_EQ(map.getResolution(), 1.0F);
}