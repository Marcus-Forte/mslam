#include "SyntheticScene.hh"

#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/NormalEstimator.hh"
#include "mslam/utils/Parallel.hh"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
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

// Bitwise equality of two optional normals.
bool sameNormal(const std::optional<Eigen::Vector3d> &a,
                const std::optional<Eigen::Vector3d> &b) {
  if (a.has_value() != b.has_value()) {
    return false;
  }
  if (!a.has_value()) {
    return true;
  }
  for (int i = 0; i < 3; ++i) {
    if (std::bit_cast<std::uint64_t>((*a)[i]) !=
        std::bit_cast<std::uint64_t>((*b)[i])) {
      return false;
    }
  }
  return true;
}

// Correspondences whose map points are @p map_points, each listed twice (the
// second time in reverse order), as several scan points matching one map point
// would be. The scan side is irrelevant to the normals.
mslam::Correspondences withDuplicates(const PointCloud &map_points) {
  mslam::Correspondences correspondences;
  for (const auto &point : map_points) {
    correspondences.emplace_back(Point{}, point);
  }
  for (auto it = map_points.rbegin(); it != map_points.rend(); ++it) {
    correspondences.emplace_back(Point{}, *it);
  }
  return correspondences;
}

// estimateAll() on 4 threads gives, for every correspondence, the normal the
// per-point estimate() gives, bit for bit, including the points without one.
TEST(NormalEstimator, EstimateAllMatchesPerPointEstimates) {
  VoxelHashMap map(0.1F, 5);
  PointCloud cloud = test::sampleRoom(200.0F, 3, 2.0F);
  // Isolated points: no normal.
  cloud.emplace_back(20.05F, 20.05F, 20.05F);
  cloud.emplace_back(-20.05F, 20.05F, 20.05F);
  map.addScan(cloud);
  const PointCloud map_points = map.getPointCloudRepresentation();
  ASSERT_GT(map_points.size(), 1000U);

  // Two batches: the second overlaps the first (cache reuse) and adds points.
  const std::size_t half = map_points.size() / 2;
  const PointCloud first(map_points.begin(),
                         map_points.begin() +
                             static_cast<std::ptrdiff_t>(half));
  const PointCloud second(map_points.begin() +
                              static_cast<std::ptrdiff_t>(half / 2),
                          map_points.end());

  NormalEstimator reference;
  reference.reset(map);
  NormalEstimator parallel;
  parallel.reset(map);
  const mslam::ParallelExecutor executor(4);
  std::vector<std::uint32_t> slots;

  std::size_t without_normal = 0;
  for (const auto *batch : {&first, &second}) {
    const auto correspondences = withDuplicates(*batch);
    parallel.estimateAll(correspondences, slots, &executor);
    ASSERT_EQ(slots.size(), correspondences.size());
    for (std::size_t i = 0; i < correspondences.size(); ++i) {
      const auto expected = reference.estimate(correspondences[i].second);
      without_normal += expected.has_value() ? 0 : 1;
      ASSERT_TRUE(sameNormal(parallel.normal(slots[i]), expected))
          << "correspondence " << i;
    }
    EXPECT_EQ(parallel.cachedCount(), reference.cachedCount());
  }
  EXPECT_EQ(parallel.cachedCount(), map_points.size());
  EXPECT_GT(without_normal, 0U);

  // estimate() reads the same cache: no new entry, same value.
  const auto again = parallel.estimate(map_points.front());
  EXPECT_EQ(parallel.cachedCount(), map_points.size());
  EXPECT_TRUE(sameNormal(again, reference.estimate(map_points.front())));
}

TEST(NormalEstimator, EstimateAllWithoutExecutorAndAfterReset) {
  VoxelHashMap map(0.1F, 5);
  map.addScan(flatGrid(0.25F));
  NormalEstimator estimator;
  estimator.reset(map);

  std::vector<std::uint32_t> slots{7, 7, 7, 7};
  estimator.estimateAll({}, slots);
  EXPECT_TRUE(slots.empty());

  const mslam::Correspondences correspondences{
      {Point{}, Point{0.25F, 0.25F, 0.25F}},
      {Point{}, Point{0.25F, 0.25F, 0.25F}}};
  estimator.estimateAll(correspondences, slots);
  ASSERT_EQ(slots.size(), 2U);
  EXPECT_EQ(slots[0], slots[1]);
  EXPECT_EQ(estimator.cachedCount(), 1U);
  ASSERT_TRUE(estimator.normal(slots[0]).has_value());
  EXPECT_NEAR(std::abs(estimator.normal(slots[0])->z()), 1.0, 1e-6);

  estimator.reset(map);
  EXPECT_EQ(estimator.cachedCount(), 0U);
  estimator.estimateAll(correspondences, slots);
  EXPECT_EQ(slots[0], 0U);
}

} // namespace
