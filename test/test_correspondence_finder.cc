#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/CorrespondenceFinder.hh"
#include <array>
#include <bit>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <random>

namespace mslam {
namespace {

class TestMap : public IMap {
public:
  PointCloud addScan(const PointCloud &scan) override {
    points_.insert(points_.end(), scan.begin(), scan.end());
    return scan;
  }

  Neighbor getClosestNeighbor(const Point &query) const override {
    Neighbor nearest{Point{}, std::numeric_limits<float>::max()};
    for (const auto &point : points_) {
      const float dx = query.x - point.x;
      const float dy = query.y - point.y;
      const float dz = query.z - point.z;
      const float distance_squared = dx * dx + dy * dy + dz * dz;
      if (distance_squared < nearest.second) {
        nearest = {point, distance_squared};
      }
    }
    return nearest;
  }

  std::size_t findClosestNeighbors(const Point &,
                                   std::span<Neighbor>) const override {
    return 0;
  }

  const PointCloud &getPointCloudRepresentation() const override {
    return points_;
  }

  void clear() override { points_.clear(); }

private:
  PointCloud points_;
};

class RecordingObserver : public ICorrespondenceFinderObserver {
public:
  void onCorrespondenceSearch(const CorrespondenceSearchEvent &event) override {
    events.push_back(event);
  }

  std::vector<CorrespondenceSearchEvent> events;
};

TEST(CorrespondenceFinderTest, EmitsSearchEventWithResultCounts) {
  TestMap map;
  map.addScan({Point(0, 0, 0)});
  PointCloud scan{Point(0, 0, 0), Point(0.5, 0, 0), Point(3, 0, 0)};

  CorrespondenceFinder finder;
  auto observer = std::make_shared<RecordingObserver>();
  finder.setObserver(observer);

  Correspondences correspondences;
  finder.find(map, scan, 1.0F, correspondences);

  ASSERT_EQ(observer->events.size(), 1);
  EXPECT_EQ(correspondences.size(), 2);
  EXPECT_EQ(observer->events[0].scan_size, scan.size());
  EXPECT_EQ(observer->events[0].correspondence_count, correspondences.size());
  EXPECT_FLOAT_EQ(observer->events[0].max_correspondence_distance, 1.0F);
  EXPECT_GE(observer->events[0].elapsed.count(), 0);
}

// Bitwise, so that -0.0 / 0.0 or a NaN cannot hide a difference.
bool sameBits(const Point &a, const Point &b) {
  return std::bit_cast<std::array<std::uint32_t, 4>>(a) ==
         std::bit_cast<std::array<std::uint32_t, 4>>(b);
}

void expectSameCorrespondences(const Correspondences &actual,
                               const Correspondences &expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    ASSERT_TRUE(sameBits(actual[i].first, expected[i].first)) << "index " << i;
    ASSERT_TRUE(sameBits(actual[i].second, expected[i].second))
        << "index " << i;
  }
}

PointCloud randomCloud(std::size_t size, unsigned seed, float extent) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> along(-extent, extent);
  PointCloud cloud;
  cloud.reserve(size);
  for (std::size_t i = 0; i < size; ++i) {
    cloud.emplace_back(along(rng), along(rng), along(rng),
                       static_cast<float>(i));
  }
  return cloud;
}

class ParallelCorrespondenceFinder : public ::testing::Test {
protected:
  void SetUp() override {
    if (!ParallelExecutor::available()) {
      GTEST_SKIP() << "built without TBB";
    }
    map_.addScan(randomCloud(4000, 1, 2.0F));
  }

  Correspondences findWith(const std::shared_ptr<const ParallelExecutor> &exec,
                           const PointCloud &scan, float gate) const {
    const CorrespondenceFinder finder(exec);
    Correspondences out;
    finder.find(map_, scan, gate, out);
    return out;
  }

  VoxelHashMap map_{0.1F, 5};
  std::shared_ptr<const ParallelExecutor> four_ =
      std::make_shared<const ParallelExecutor>(4);
};

// The scan reaches past the map, so some points are rejected by the gate and
// the compaction has gaps to close.
TEST_F(ParallelCorrespondenceFinder, MatchesTheSequentialSearch) {
  const PointCloud scan = randomCloud(3000, 2, 2.5F);
  const auto expected = findWith(nullptr, scan, 0.08F);
  ASSERT_GT(expected.size(), 0U);
  ASSERT_LT(expected.size(), scan.size());
  expectSameCorrespondences(findWith(four_, scan, 0.08F), expected);
}

TEST_F(ParallelCorrespondenceFinder, IsReusableWithSmallerScans) {
  const CorrespondenceFinder finder(four_);
  Correspondences out;
  finder.find(map_, randomCloud(3000, 3, 2.0F), 0.1F, out);
  const PointCloud small = randomCloud(1000, 4, 2.0F);
  finder.find(map_, small, 0.1F, out);
  expectSameCorrespondences(out, findWith(nullptr, small, 0.1F));
}

TEST_F(ParallelCorrespondenceFinder, EmptyScanGivesNoCorrespondences) {
  EXPECT_TRUE(findWith(four_, PointCloud{}, 0.1F).empty());
}

TEST_F(ParallelCorrespondenceFinder, AllRejectedByTheGate) {
  // Far outside the map: every nearest neighbour is either missing or too far.
  PointCloud scan = randomCloud(2000, 5, 1.0F);
  for (auto &point : scan) {
    point.x += 50.0F;
  }
  EXPECT_TRUE(findWith(four_, scan, 0.1F).empty());
}

TEST_F(ParallelCorrespondenceFinder, BelowTheInlineThreshold) {
  const PointCloud scan = randomCloud(20, 6, 2.0F);
  expectSameCorrespondences(findWith(four_, scan, 0.1F),
                            findWith(nullptr, scan, 0.1F));
}

} // namespace
} // namespace mslam
