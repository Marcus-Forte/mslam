#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/NormalEstimator.hh"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

namespace {

using mslam::IMap;
using mslam::NormalEstimator;
using mslam::Point;
using mslam::PointCloud;
using mslam::VoxelHashMap;

// Reference normal: smallest-eigenvalue eigenvector from the iterative solver.
Eigen::Vector3d iterativeNormal(const std::vector<IMap::Neighbor> &neighbors) {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto &[point, _] : neighbors) {
    centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  centroid /= static_cast<double>(neighbors.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &[point, _] : neighbors) {
    const Eigen::Vector3d centered =
        Eigen::Vector3d(point.x, point.y, point.z) - centroid;
    covariance += centered * centered.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  return solver.eigenvectors().col(0);
}

std::vector<IMap::Neighbor> planarNeighbors(std::mt19937 &rng, double noise,
                                            double spread, double offset) {
  std::normal_distribution<double> gaussian(0.0, 1.0);
  Eigen::Vector3d normal(gaussian(rng), gaussian(rng), gaussian(rng));
  normal.normalize();
  const Eigen::Vector3d u = normal.unitOrthogonal();
  const Eigen::Vector3d v = normal.cross(u);
  const Eigen::Vector3d origin =
      offset * Eigen::Vector3d(gaussian(rng), gaussian(rng), gaussian(rng));

  std::vector<IMap::Neighbor> neighbors;
  for (int i = 0; i < 5; ++i) {
    const Eigen::Vector3d p = origin + spread * gaussian(rng) * u +
                              spread * gaussian(rng) * v +
                              noise * gaussian(rng) * normal;
    neighbors.emplace_back(Point{static_cast<float>(p.x()),
                                 static_cast<float>(p.y()),
                                 static_cast<float>(p.z())},
                           0.0F);
  }
  return neighbors;
}

TEST(NormalEstimator, ClosedFormSolverMatchesIterativeSolverOnPlanarPatches) {
  std::mt19937 rng(17);
  for (const double noise : {0.0, 1e-3, 1e-2}) {
    for (const double offset : {0.0, 5.0, 50.0}) {
      for (int trial = 0; trial < 500; ++trial) {
        const auto neighbors = planarNeighbors(rng, noise, 0.1, offset);
        const auto normal = NormalEstimator::fromNeighbors(neighbors);
        ASSERT_TRUE(normal.has_value());
        EXPECT_NEAR(normal->norm(), 1.0, 1e-9);
        // Normals are defined up to sign.
        const double alignment =
            std::abs(normal->dot(iterativeNormal(neighbors)));
        EXPECT_GT(alignment, 1.0 - 1e-6)
            << "noise " << noise << " offset " << offset;
      }
    }
  }
}

TEST(NormalEstimator, TooFewNeighborsGiveNoNormal) {
  std::vector<IMap::Neighbor> two{{Point{0, 0, 0}, 0.0F},
                                  {Point{1, 0, 0}, 1.0F}};
  EXPECT_FALSE(NormalEstimator::fromNeighbors(two).has_value());
}

PointCloud flatGrid(float z) {
  PointCloud cloud;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      cloud.emplace_back(0.05F + 0.1F * static_cast<float>(x),
                         0.05F + 0.1F * static_cast<float>(y), z);
    }
  }
  return cloud;
}

TEST(NormalEstimator, EstimatesPlaneNormalFromMap) {
  VoxelHashMap map(0.1F, 5);
  map.addScan(flatGrid(0.25F));

  NormalEstimator estimator;
  estimator.reset(map);
  const auto normal = estimator.estimate(Point{0.25F, 0.25F, 0.25F});
  ASSERT_TRUE(normal.has_value());
  EXPECT_NEAR(std::abs(normal->z()), 1.0, 1e-6);
}

TEST(NormalEstimator, CachesPerPointAndResetDropsTheCache) {
  VoxelHashMap map(0.1F, 5);
  map.addScan(flatGrid(0.25F));

  NormalEstimator estimator;
  estimator.reset(map);
  const Point query{0.25F, 0.25F, 0.25F};
  const auto first = estimator.estimate(query);
  EXPECT_EQ(estimator.cachedCount(), 1U);
  const auto second = estimator.estimate(query);
  EXPECT_EQ(estimator.cachedCount(), 1U);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, *second);

  // A map change is only seen after reset(): the cache is never carried over.
  VoxelHashMap tilted(0.1F, 5);
  PointCloud wall;
  for (int x = 0; x < 5; ++x) {
    for (int z = 0; z < 5; ++z) {
      wall.emplace_back(0.05F + 0.1F * static_cast<float>(x), 0.25F,
                        0.05F + 0.1F * static_cast<float>(z));
    }
  }
  tilted.addScan(wall);
  estimator.reset(tilted);
  EXPECT_EQ(estimator.cachedCount(), 0U);
  const auto third = estimator.estimate(Point{0.25F, 0.25F, 0.25F});
  ASSERT_TRUE(third.has_value());
  EXPECT_NEAR(std::abs(third->y()), 1.0, 1e-6);
}

TEST(NormalEstimator, DistinguishesSignedZeroKeysWithoutFailing) {
  VoxelHashMap map(0.1F, 5);
  PointCloud cloud = flatGrid(0.0F);
  map.addScan(cloud);
  NormalEstimator estimator;
  estimator.reset(map);
  const auto positive = estimator.estimate(Point{0.05F, 0.05F, 0.0F});
  const auto negative = estimator.estimate(Point{0.05F, 0.05F, -0.0F});
  ASSERT_TRUE(positive.has_value());
  ASSERT_TRUE(negative.has_value());
  EXPECT_NEAR(std::abs(positive->dot(*negative)), 1.0, 1e-9);
}

TEST(NormalEstimator, IsolatedPointHasNoNormal) {
  VoxelHashMap map(0.1F, 5);
  PointCloud cloud;
  cloud.emplace_back(0.05F, 0.05F, 0.05F);
  map.addScan(cloud);
  NormalEstimator estimator;
  estimator.reset(map);
  EXPECT_FALSE(estimator.estimate(Point{0.05F, 0.05F, 0.05F}).has_value());
}

} // namespace
