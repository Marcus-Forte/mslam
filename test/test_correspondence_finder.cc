#include "slam/CorrespondenceFinder.hh"
#include <gtest/gtest.h>
#include <limits>

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

  std::vector<Neighbor> getClosestNNeighbors(const Point &,
                                             int) const override {
    return {};
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

} // namespace
} // namespace mslam
