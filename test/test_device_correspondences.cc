// The SYCL device path of the registrations: SyclVoxelHashMap's mirror, the
// device searches and the correspondence kernel, against the host VoxelHashMap,
// CorrespondenceFinder and NormalEstimator they replace.
#include "SyntheticScene.hh"

#include "mslam/map/DeviceVoxelSearch.hh"
#include "mslam/map/SyclVoxelHashMap.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/CorrespondenceFinder.hh"
#include "mslam/slam/NormalEstimator.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/DeviceCorrespondenceSearch.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"
#include "mslam/utils/Parallel.hh"

#include <array>
#include <bit>
#include <limits>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <random>
#include <spdlog/spdlog.h>

namespace {

std::shared_ptr<spdlog::logger> silentLogger() {
  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);
  return logger;
}

bool sameBits(float a, float b) {
  return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
}

// A room seen from a few poses: incremental scans that fill some voxels up.
std::vector<mslam::PointCloud> roomScans(int count) {
  std::vector<mslam::PointCloud> scans;
  for (int i = 0; i < count; ++i) {
    mslam::PointCloud scan =
        test::sampleRoom(120.0F, static_cast<unsigned>(10 + i), 2.5F);
    mslam::transformCloud(
        mslam::toAffine(0.03 * i, -0.02 * i, 0.01 * i, 0.0, 0.0, 0.02 * i),
        scan);
    scans.push_back(std::move(scan));
  }
  return scans;
}

mslam::PointCloud queriesNear(const mslam::PointCloud &cloud, unsigned seed,
                              float spread) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> noise(0.0F, spread);
  mslam::PointCloud queries;
  for (std::size_t i = 0; i < cloud.size(); i += 3) {
    queries.emplace_back(cloud[i].x + noise(rng), cloud[i].y + noise(rng),
                         cloud[i].z + noise(rng), 1.0F);
  }
  queries.emplace_back(100.0F, 100.0F, 100.0F, 1.0F); // empty neighbourhood
  return queries;
}

// The device searches on the mirror against the host queries, bit for bit.
void expectSameSearches(const mslam::SyclVoxelHashMap &map,
                        const mslam::PointCloud &queries) {
  const mslam::device::MapView view = map.view();
  for (const auto &q : queries) {
    const auto host = map.getClosestNeighbor(q);
    const auto dev = mslam::device::closestNeighbor(view, q.x, q.y, q.z);
    ASSERT_TRUE(sameBits(host.second, dev.squared_distance));
    if (dev.slot != std::numeric_limits<std::uint32_t>::max()) {
      const auto &p = view.pool[dev.slot];
      ASSERT_TRUE(sameBits(p.x, host.first.x) && sameBits(p.y, host.first.y) &&
                  sameBits(p.z, host.first.z));
    }

    std::array<mslam::IMap::Neighbor, 5> host_knn;
    const auto host_count = map.findClosestNeighbors(q, host_knn);
    std::uint32_t slots[5];
    float distances[5];
    const auto dev_count = mslam::device::closestNeighbors(view, q.x, q.y, q.z,
                                                           5, slots, distances);
    ASSERT_EQ(host_count, dev_count);
    for (std::size_t k = 0; k < host_count; ++k) {
      ASSERT_TRUE(sameBits(host_knn[k].second, distances[k]));
      const auto &p = view.pool[slots[k]];
      ASSERT_TRUE(sameBits(p.x, host_knn[k].first.x) &&
                  sameBits(p.y, host_knn[k].first.y) &&
                  sameBits(p.z, host_knn[k].first.z));
    }
  }
}

} // namespace

TEST(SyclVoxelHashMap, DeviceSearchesEqualTheHostQueriesAsScansAreAdded) {
  for (const int radius : {1, 2}) {
    SCOPED_TRACE(testing::Message() << "radius=" << radius);
    mslam::SyclVoxelHashMap map(0.1F, 5);
    map.setNumAdjacentVoxelSearch(radius);
    const auto scans = roomScans(6);
    for (std::size_t i = 0; i < scans.size(); ++i) {
      map.addScan(scans[i]);
      expectSameSearches(map, queriesNear(scans[i], 100 + i, 0.03F));
    }
  }
}

TEST(SyclVoxelHashMap, PruneAndClearRebuildTheMirror) {
  mslam::SyclVoxelHashMap map(0.1F, 5);
  for (const auto &scan : roomScans(4)) {
    map.addScan(scan);
  }
  const auto queries = queriesNear(roomScans(1)[0], 7, 0.05F);
  map.prune(mslam::Point{0.0F, 0.0F, 0.0F, 0.0F}, 1.5F, 0);
  expectSameSearches(map, queries);
  map.clear();
  EXPECT_EQ(map.size(), 0U);
  expectSameSearches(map, queries);
  map.addScan(roomScans(2)[1]);
  expectSameSearches(map, queries);
}

TEST(DeviceCorrespondenceSearch, MatchesTheHostFinderAndNormalEstimator) {
  mslam::SyclVoxelHashMap map(0.1F, 5);
  mslam::VoxelHashMap host_map(0.1F, 5);
  for (const auto &scan : roomScans(3)) {
    map.addScan(scan);
    host_map.addScan(scan);
  }
  mslam::PointCloud source = queriesNear(roomScans(5)[4], 3, 0.02F);

  mslam::Correspondences expected;
  mslam::CorrespondenceFinder().find(host_map, source, 0.2F, expected);
  mslam::NormalEstimator estimator;
  estimator.reset(host_map);

  mslam::DeviceCorrespondenceSearch search;
  const mslam::DeviceCorrespondence *found = search.run(map, source, 0.2F, 3);
  std::size_t matched = 0;
  for (std::size_t i = 0; i < source.size(); ++i) {
    if (found[i].matched == 0) {
      continue;
    }
    ASSERT_LT(matched, expected.size());
    const auto &[scan_point, map_point] = expected[matched++];
    EXPECT_TRUE(sameBits(scan_point.x, source[i].x));
    EXPECT_TRUE(sameBits(map_point.x, found[i].map_point[0]) &&
                sameBits(map_point.y, found[i].map_point[1]) &&
                sameBits(map_point.z, found[i].map_point[2]));
    ASSERT_EQ(found[i].computed, 1); // empty cache: every normal is computed
    const auto normal = estimator.estimate(map_point);
    ASSERT_EQ(found[i].has_normal != 0, normal.has_value());
    if (normal) {
      // The host pass vectorizes Eigen, the device pass does not.
      for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(found[i].normal[k], (*normal)[k], 1e-12);
      }
    }
  }
  EXPECT_EQ(matched, expected.size());
}

TEST(DeviceCorrespondenceSearch, ResultsDoNotDependOnTheWidth) {
  mslam::SyclVoxelHashMap map(0.1F, 5);
  for (const auto &scan : roomScans(3)) {
    map.addScan(scan);
  }
  const mslam::PointCloud source = queriesNear(roomScans(4)[3], 5, 0.02F);
  mslam::DeviceCorrespondenceSearch search;
  const auto *reference = search.run(map, source, 0.2F, 1);
  std::vector<mslam::DeviceCorrespondence> first(reference,
                                                 reference + source.size());
  for (const std::size_t width : {2, 3, 11, 64}) {
    const auto *found = search.run(map, source, 0.2F, width);
    for (std::size_t i = 0; i < source.size(); ++i) {
      ASSERT_EQ(first[i].matched, found[i].matched);
      if (first[i].matched == 0) {
        continue;
      }
      ASSERT_EQ(first[i].slot, found[i].slot);
      ASSERT_EQ(first[i].has_normal, found[i].has_normal);
      ASSERT_EQ(std::memcmp(first[i].normal, found[i].normal, sizeof(double) * 3),
                0);
    }
  }
}

// The cache is exact: after scans are added, a cached normal is what a map that
// never cached anything computes for the same point.
TEST(DeviceCorrespondenceSearch, CachedNormalsEqualFreshOnesAfterTheMapChanges) {
  const auto scans = roomScans(6);
  mslam::SyclVoxelHashMap cached(0.1F, 5);
  mslam::IcpWorkspace workspace(mslam::IcpParameters{1, 1, 0.2F},
                                silentLogger(),
                                std::make_shared<mslam::CorrespondenceFinder>());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    cached.addScan(scans[i]);
    // Fill the cache with this scan's correspondences, as a registration does.
    const mslam::PointCloud source = queriesNear(scans[i], 50 + i, 0.02F);
    workspace.begin(cached, source, Eigen::Affine3d::Identity());
    workspace.findPlaneConstraints(cached);
  }
  ASSERT_GT(cached.cachedNormals(), 0U);

  mslam::SyclVoxelHashMap fresh(0.1F, 5);
  for (const auto &scan : scans) {
    fresh.addScan(scan);
  }
  const mslam::PointCloud source = queriesNear(scans[2], 99, 0.02F);
  mslam::DeviceCorrespondenceSearch a, b;
  const auto *from_cache = a.run(cached, source, 0.2F, 4);
  const auto *computed = b.run(fresh, source, 0.2F, 4);
  std::size_t hits = 0;
  for (std::size_t i = 0; i < source.size(); ++i) {
    ASSERT_EQ(from_cache[i].matched, computed[i].matched);
    if (computed[i].matched == 0) {
      continue;
    }
    hits += from_cache[i].computed == 0 ? 1 : 0;
    ASSERT_EQ(from_cache[i].has_normal, computed[i].has_normal);
    if (computed[i].has_normal != 0) {
      ASSERT_EQ(std::memcmp(from_cache[i].normal, computed[i].normal,
                            sizeof(double) * 3),
                0);
    }
  }
  EXPECT_GT(hits, 0U); // the test exercised the cache
}

TEST(IcpWorkspace, DevicePathAssemblesTheHostPathsConstraints) {
  mslam::SyclVoxelHashMap device_map(0.1F, 5);
  mslam::VoxelHashMap host_map(0.1F, 5);
  for (const auto &scan : roomScans(3)) {
    device_map.addScan(scan);
    host_map.addScan(scan);
  }
  const mslam::PointCloud scan = queriesNear(roomScans(4)[3], 11, 0.02F);
  const Eigen::Affine3d pose = mslam::toAffine(0.01, 0.0, -0.01, 0.0, 0.01, 0.0);

  auto make = [] {
    return mslam::IcpWorkspace(mslam::IcpParameters{1, 1, 0.2F}, silentLogger(),
                               std::make_shared<mslam::CorrespondenceFinder>());
  };
  mslam::IcpWorkspace device = make();
  mslam::IcpWorkspace host = make();
  device.begin(device_map, scan, pose);
  host.begin(host_map, scan, pose);
  const std::size_t n = device.findPlaneConstraints(device_map);
  ASSERT_EQ(n, host.findPlaneConstraints(host_map));
  ASSERT_EQ(device.correspondences().size(), host.correspondences().size());
  for (std::size_t i = 0; i < n; ++i) {
    for (int k = 0; k < 3; ++k) {
      EXPECT_EQ(device.planeInputs()[i][k], host.planeInputs()[i][k]);
      EXPECT_NEAR(device.planeInputs()[i][3 + k], host.planeInputs()[i][3 + k],
                  1e-12);
    }
    EXPECT_NEAR(device.planeOffsets()[i], host.planeOffsets()[i], 1e-12);
  }
}
