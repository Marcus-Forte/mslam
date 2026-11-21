#include "mslam/slam/posegraph/KeyframeBuilder.hh"
#include "mslam/slam/posegraph/PoseGraph.hh"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <set>
#include <vector>

namespace {

using mslam::KeyframeBuilder;
using mslam::PointCloud;
using mslam::PoseGraphParameters;

constexpr std::int64_t kStep = 100'000'000; // 0.1 s between scans

PoseGraphParameters parameters(double distance, double angle,
                               double interval_s) {
  PoseGraphParameters p;
  p.keyframe_distance = distance;
  p.keyframe_angle = angle;
  p.keyframe_max_interval_s = interval_s;
  p.mini_map_voxel_size = 0.05;
  return p;
}

Eigen::Affine3d poseAt(double x, double yaw = 0.0) {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, 0.0, 0.0);
  return pose;
}

PointCloud onePoint(double x, double y, double z) {
  PointCloud cloud;
  cloud.push_back({static_cast<float>(x), static_cast<float>(y),
                   static_cast<float>(z), 10.0F});
  return cloud;
}

// Indices of the scans that sealed a keyframe, for scans at the given poses.
std::vector<int> sealingScans(KeyframeBuilder &builder,
                              const std::vector<Eigen::Affine3d> &poses,
                              std::vector<std::size_t> *counts = nullptr) {
  std::vector<int> sealed;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    if (const auto done = builder.onScan(static_cast<std::int64_t>(k) * kStep,
                                         poses[k], onePoint(1.0, 0.0, 0.0))) {
      sealed.push_back(static_cast<int>(k));
      if (counts != nullptr) {
        counts->push_back(done->scan_count);
      }
    }
  }
  return sealed;
}

TEST(KeyframeBuilder, DistanceOpensAKeyframeBeyondTheThreshold) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
  std::vector<Eigen::Affine3d> poses;
  for (int k = 0; k < 14; ++k) {
    poses.push_back(poseAt(0.1 * k)); // 0.5 m at scan 5 is not "more than"
  }
  std::vector<std::size_t> counts;
  const auto sealed = sealingScans(builder, poses, &counts);
  // 0.6 m at scan 6 opens keyframe 1 (the first one holds scans 0 to 5), and
  // keyframe 2 opens at scan 12 (0.6 m from scan 6).
  EXPECT_EQ(sealed, (std::vector<int>{6, 12}));
  EXPECT_EQ(counts, (std::vector<std::size_t>{6, 6}));
}

TEST(KeyframeBuilder, AngleOpensAKeyframeBeyondTheThreshold) {
  KeyframeBuilder builder(parameters(100.0, 0.26, 0.0));
  std::vector<Eigen::Affine3d> poses;
  for (int k = 0; k < 8; ++k) {
    poses.push_back(poseAt(0.0, 0.1 * k)); // 0.3 rad at scan 3
  }
  EXPECT_EQ(sealingScans(builder, poses), (std::vector<int>{3, 6}));
}

TEST(KeyframeBuilder, IntervalClosesAKeyframeWhileStanding) {
  KeyframeBuilder builder(parameters(0.5, 0.26, 1.0));
  const std::vector<Eigen::Affine3d> still(35, poseAt(0.0));
  // 1.0 s = 10 scans after the keyframe opened.
  EXPECT_EQ(sealingScans(builder, still), (std::vector<int>{10, 20, 30}));
}

TEST(KeyframeBuilder, NothingOpensWhileStillWithoutAnInterval) {
  KeyframeBuilder builder(parameters(0.5, 0.26, 0.0));
  const std::vector<Eigen::Affine3d> still(500, poseAt(0.0));
  EXPECT_TRUE(sealingScans(builder, still).empty());
  EXPECT_EQ(builder.openScanCount(), 500U);
}

TEST(KeyframeBuilder, MiniMapHoldsEveryScanInTheKeyframesFrame) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
  // Keyframe 0 at x = 0; its scans see one point each in the odom frame.
  // Keyframe 1 opens at scan 3 (x = 0.6) and is turned by 90 degrees.
  const Eigen::Affine3d pose_kf1 = poseAt(0.6, M_PI / 2.0);
  const std::vector<Eigen::Affine3d> poses = {
      poseAt(0.0), poseAt(0.2), poseAt(0.4), pose_kf1, poseAt(0.6, M_PI / 2.0)};
  // Distinct points far apart (> voxel), so none merge.
  const std::vector<Eigen::Vector3d> odom_points = {{1.0, 0.0, 0.0},
                                                    {2.0, 0.5, 0.0},
                                                    {3.0, 1.0, 0.5},
                                                    {4.0, 1.5, 0.0},
                                                    {5.0, 2.0, 0.0}};

  std::vector<mslam::SealedKeyframe> sealed;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    const auto &p = odom_points[k];
    if (auto done = builder.onScan(static_cast<std::int64_t>(k) * kStep,
                                   poses[k], onePoint(p.x(), p.y(), p.z()))) {
      sealed.push_back(std::move(*done));
    }
  }
  ASSERT_EQ(sealed.size(), 1U); // sealed exactly once
  EXPECT_EQ(sealed[0].scan_count, 3U);
  EXPECT_TRUE(sealed[0].pose_odom.matrix() == poseAt(0.0).matrix());
  ASSERT_TRUE(sealed[0].mini_map);
  ASSERT_EQ(sealed[0].mini_map->size(), 3U);
  for (std::size_t k = 0; k < 3; ++k) {
    const Eigen::Vector3d expected = odom_points[k]; // keyframe 0 = odom
    const auto &p = (*sealed[0].mini_map)[k];
    EXPECT_NEAR(p.x, expected.x(), 1e-6);
    EXPECT_NEAR(p.y, expected.y(), 1e-6);
    EXPECT_NEAR(p.z, expected.z(), 1e-6);
  }

  // The open mini map: the scan that opened keyframe 1 and the next one, in
  // keyframe 1's frame.
  EXPECT_TRUE(builder.hasOpenKeyframe());
  EXPECT_EQ(builder.openScanCount(), 2U);
  const PointCloud open = builder.openMiniMap();
  ASSERT_EQ(open.size(), 2U);
  for (std::size_t k = 0; k < 2; ++k) {
    const Eigen::Vector3d expected = pose_kf1.inverse() * odom_points[3 + k];
    EXPECT_NEAR(open[k].x, expected.x(), 1e-5);
    EXPECT_NEAR(open[k].y, expected.y(), 1e-5);
    EXPECT_NEAR(open[k].z, expected.z(), 1e-5);
  }
}

TEST(KeyframeBuilder, ThinningStaysBoundedAsScansAccumulate) {
  KeyframeBuilder builder(parameters(100.0, 3.0, 0.0));
  PointCloud scan;
  for (int i = 0; i < 100; ++i) {
    scan.push_back({0.013F * static_cast<float>(i), 0.0F, 0.0F, 1.0F});
  }
  builder.onScan(0, poseAt(0.0), scan);
  const std::size_t after_one = builder.openMiniMap().size();
  for (int k = 1; k < 200; ++k) {
    builder.onScan(k * kStep, poseAt(0.0), scan);
  }
  EXPECT_EQ(builder.openMiniMap().size(), after_one);
  EXPECT_LT(after_one, 100U); // 0.05 m voxels merge the 1.3 cm spacing
  EXPECT_EQ(builder.openScanCount(), 200U);
}

TEST(KeyframeBuilder, ResetStartsOver) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
  builder.onScan(0, poseAt(0.0), onePoint(1, 0, 0));
  builder.reset();
  EXPECT_FALSE(builder.hasOpenKeyframe());
  // The first scan after a reset opens a keyframe without sealing anything.
  EXPECT_FALSE(
      builder.onScan(kStep, poseAt(9.0), onePoint(1, 0, 0)).has_value());
  EXPECT_EQ(builder.openScanCount(), 1U);
}

// ---------------------------------------------------------------------------
// Voxel ownership: a voxel belongs to the first keyframe that observes it.

// Points on the wall x = 3 spaced 0.2 m (> the 0.05 m voxel), `count` of them
// starting at y = first_y; and a second wall at y = 3 for something new.
PointCloud wall(int count, double first_y, double x = 3.0) {
  // Offsets keep every point in the middle of its voxel, so that moving it
  // between frames in float cannot tip it into the neighbour.
  PointCloud cloud;
  for (int i = 0; i < count; ++i) {
    cloud.push_back({static_cast<float>(x + 0.013),
                     static_cast<float>(first_y + 0.2 * i + 0.017),
                     0.5F + 0.013F, 10.0F});
  }
  return cloud;
}

PointCloud joined(PointCloud a, const PointCloud &b) {
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

std::set<std::array<int, 3>> odomVoxels(const PointCloud &cloud,
                                        const Eigen::Affine3d &odom_from_kf,
                                        double voxel = 0.05) {
  std::set<std::array<int, 3>> voxels;
  for (const auto &point : cloud) {
    const Eigen::Vector3d p =
        odom_from_kf * Eigen::Vector3d(point.x, point.y, point.z);
    voxels.insert({static_cast<int>(std::floor(p.x() / voxel)),
                   static_cast<int>(std::floor(p.y() / voxel)),
                   static_cast<int>(std::floor(p.z() / voxel))});
  }
  return voxels;
}

TEST(OwnedMapBuilder,
     TwoKeyframesSeeingTheSameWallOwnItOnceAndBothViewsHoldIt) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
  const PointCloud wall_a = wall(20, 0.0);      // seen by both
  const PointCloud wall_b = wall(10, 0.0, 4.0); // only keyframe 1 sees it
  const Eigen::Affine3d pose1 = poseAt(0.6, 0.3);

  EXPECT_FALSE(builder.onScan(0, poseAt(0.0), wall_a).has_value());
  auto sealed0 = builder.onScan(kStep, pose1, joined(wall_a, wall_b));
  ASSERT_TRUE(sealed0.has_value());
  const auto sealed1 = builder.finish();
  ASSERT_TRUE(sealed1.has_value());

  // The first keyframe owns the whole wall; the second owns only what is new.
  EXPECT_EQ(sealed0->mini_map->size(), wall_a.size());
  EXPECT_EQ(sealed1->mini_map->size(), wall_b.size());
  // The union has one point per odom voxel: no voxel in both.
  const auto owned0 = odomVoxels(*sealed0->mini_map, sealed0->pose_odom);
  const auto owned1 = odomVoxels(*sealed1->mini_map, sealed1->pose_odom);
  EXPECT_EQ(owned0.size(), wall_a.size());
  EXPECT_EQ(owned1.size(), wall_b.size());
  for (const auto &voxel : owned1) {
    EXPECT_EQ(owned0.count(voxel), 0U);
  }
  // Both views still hold the wall (what the loop verification matches).
  EXPECT_EQ(sealed0->view->size(), wall_a.size());
  EXPECT_EQ(sealed1->view->size(), wall_a.size() + wall_b.size());
  const auto view1 = odomVoxels(*sealed1->view, sealed1->pose_odom);
  for (const auto &voxel : owned0) {
    EXPECT_EQ(view1.count(voxel), 1U);
  }
}

TEST(OwnedMapBuilder, ASealedVoxelIsNeverOwnedAgain) {
  mslam::OwnedMapBuilder owned(0.05);
  owned.begin(Eigen::Affine3d::Identity());
  EXPECT_EQ(owned.add(wall(5, 0.0)).size(), 5U);
  const auto first = owned.seal();
  ASSERT_EQ(first->size(), 5U);
  EXPECT_EQ(owned.sealedVoxels(), 5U);

  owned.begin(poseAt(1.0));
  EXPECT_TRUE(owned.add(wall(5, 0.0)).empty()); // all five are frozen
  const PointCloud newcomer = wall(1, 5.0);
  EXPECT_EQ(owned.add(joined(wall(5, 0.0), newcomer)).size(), 1U);
  const auto second = owned.seal();
  ASSERT_EQ(second->size(), 1U);
  EXPECT_EQ(owned.sealedVoxels(), 6U);
  // The newcomer comes out in the second keyframe's frame (x = 1 in odom).
  EXPECT_NEAR((*second)[0].x, newcomer[0].x - 1.0, 1e-5);
  EXPECT_NEAR((*second)[0].y, newcomer[0].y, 1e-5);
}

TEST(OwnedMapBuilder, AddReturnsAVoxelOnceAndTheCentroidKeepsEveryScan) {
  mslam::OwnedMapBuilder owned(0.05);
  owned.begin(Eigen::Affine3d::Identity());
  PointCloud scan_one = {{1.00F, 1.0F, 1.0F, 10.0F},
                         {1.02F, 1.0F, 1.0F, 30.0F}};
  PointCloud scan_two = {{1.04F, 1.0F, 1.0F, 20.0F}, {2.0F, 2.0F, 2.0F, 5.0F}};

  // Two points of one voxel come back as their mean.
  const PointCloud first = owned.add(scan_one);
  ASSERT_EQ(first.size(), 1U);
  EXPECT_NEAR(first[0].x, 1.01F, 1e-6);
  EXPECT_NEAR(first[0].intensity, 20.0F, 1e-6);
  // The next scan: the voxel is already the open keyframe's, only the other
  // point is new.
  const PointCloud second = owned.add(scan_two);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_FLOAT_EQ(second[0].x, 2.0F);

  // The sealed centroid averages all three points of the first voxel.
  const auto sealed = owned.seal();
  ASSERT_EQ(sealed->size(), 2U);
  EXPECT_NEAR((*sealed)[0].x, 1.02F, 1e-6);
  EXPECT_NEAR((*sealed)[0].intensity, 20.0F, 1e-6);
}

TEST(OwnedMapBuilder, OutputIsInFirstSeenOrderAndRepeatable) {
  const auto run = [] {
    mslam::OwnedMapBuilder owned(0.05);
    owned.begin(Eigen::Affine3d::Identity());
    // Voxels first seen in a deliberately non-sorted order, over two scans.
    owned.add({{5.0F, 0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F, 1.0F}});
    owned.add({{9.0F, 0.0F, 0.0F, 1.0F},
               {1.0F, 0.0F, 0.0F, 1.0F},
               {3.0F, 0.0F, 0.0F, 1.0F}});
    return *owned.seal();
  };
  const PointCloud a = run();
  const PointCloud b = run();
  ASSERT_EQ(a.size(), 4U);
  const std::vector<float> expected = {5.0F, 1.0F, 9.0F, 3.0F};
  for (std::size_t k = 0; k < a.size(); ++k) {
    EXPECT_FLOAT_EQ(a[k].x, expected[k]);
    EXPECT_EQ(a[k].x, b[k].x); // bit for bit
  }
}

TEST(OwnedMapBuilder, KeyframeBuilderReportsTheNewlyOwnedPointsPerScan) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
  PointCloud fresh;
  builder.onScan(0, poseAt(0.0), wall(8, 0.0), &fresh);
  EXPECT_EQ(fresh.size(), 8U);
  builder.onScan(kStep, poseAt(0.1), wall(8, 0.0), &fresh);
  EXPECT_TRUE(fresh.empty()); // the same voxels, already the keyframe's
  // The scan that opens keyframe 1 sees the wall again: not new, even though
  // keyframe 0 is sealed by it.
  const auto sealed = builder.onScan(
      2 * kStep, poseAt(0.7), joined(wall(8, 0.0), wall(2, 9.0)), &fresh);
  ASSERT_TRUE(sealed.has_value());
  EXPECT_EQ(sealed->mini_map->size(), 8U);
  EXPECT_EQ(fresh.size(), 2U);
  // The open keyframe owns just those two.
  EXPECT_EQ(builder.openMiniMap().size(), 2U);
  EXPECT_EQ(builder.openView().size(), 10U);
}

TEST(OwnedMapBuilder, FinishThenTheNextScanOpensANewKeyframe) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
  for (int k = 0; k < 4; ++k) {
    builder.onScan(k * kStep, poseAt(0.0), wall(6, 0.0));
  }
  const auto origin = builder.finish();
  ASSERT_TRUE(origin.has_value());
  EXPECT_EQ(origin->scan_count, 4U);
  EXPECT_TRUE(origin->pose_odom.matrix().isIdentity(0.0));
  EXPECT_EQ(origin->mini_map->size(), 6U);
  EXPECT_FALSE(builder.hasOpenKeyframe());
  EXPECT_FALSE(builder.finish().has_value()); // nothing left to seal

  // The next scan opens keyframe 1 without sealing anything, and owns only
  // what the origin did not.
  PointCloud fresh;
  EXPECT_FALSE(builder
                   .onScan(5 * kStep, poseAt(0.0),
                           joined(wall(6, 0.0), wall(1, 7.0)), &fresh)
                   .has_value());
  EXPECT_EQ(fresh.size(), 1U);
  EXPECT_TRUE(builder.hasOpenKeyframe());
}

// ---------------------------------------------------------------------------
// Why a keyframe closed, and what it spanned.

using mslam::CloseReason;

// Feeds scans at `poses` (0.1 s apart) and returns the first keyframe sealed.
std::optional<mslam::SealedKeyframe>
firstSealed(KeyframeBuilder &builder,
            const std::vector<Eigen::Affine3d> &poses) {
  for (std::size_t k = 0; k < poses.size(); ++k) {
    if (auto sealed = builder.onScan(static_cast<std::int64_t>(k) * kStep,
                                     poses[k], onePoint(1.0, 0.0, 0.0))) {
      return sealed;
    }
  }
  return std::nullopt;
}

TEST(CloseReason, DistanceAngleAndIntervalAreReported) {
  {
    KeyframeBuilder builder(parameters(0.5, 3.0, 0.0));
    const auto sealed =
        firstSealed(builder, {poseAt(0.0), poseAt(0.3), poseAt(0.6)});
    ASSERT_TRUE(sealed.has_value());
    EXPECT_EQ(sealed->close_reason, CloseReason::Distance);
    // It held the scans at 0.0 s and 0.1 s: two scans, spanning 0.1 s.
    EXPECT_EQ(sealed->scan_count, 2U);
    EXPECT_EQ(sealed->stamp_ns, 0);
    EXPECT_EQ(sealed->last_stamp_ns, kStep);
  }
  {
    KeyframeBuilder builder(parameters(100.0, 0.26, 0.0));
    const auto sealed =
        firstSealed(builder, {poseAt(0.0), poseAt(0.0, 0.1), poseAt(0.0, 0.3)});
    ASSERT_TRUE(sealed.has_value());
    EXPECT_EQ(sealed->close_reason, CloseReason::Angle);
  }
  {
    KeyframeBuilder builder(parameters(100.0, 3.0, 0.25));
    const auto sealed = firstSealed(
        builder, {poseAt(0.0), poseAt(0.0), poseAt(0.0), poseAt(0.0)});
    ASSERT_TRUE(sealed.has_value());
    EXPECT_EQ(sealed->close_reason, CloseReason::Interval);
    EXPECT_EQ(sealed->scan_count, 3U); // 0.0, 0.1, 0.2 s; 0.3 s opened the next
    EXPECT_EQ(sealed->last_stamp_ns, 2 * kStep);
  }
}

TEST(CloseReason, DistanceIsReportedBeforeAngleWhenBothHold) {
  KeyframeBuilder builder(parameters(0.5, 0.26, 0.0));
  const auto sealed = firstSealed(builder, {poseAt(0.0), poseAt(1.0, 1.0)});
  ASSERT_TRUE(sealed.has_value());
  EXPECT_EQ(sealed->close_reason, CloseReason::Distance);
}

TEST(CloseReason, SeedAndFinishComeFromTheCaller) {
  KeyframeBuilder builder(parameters(0.5, 3.0, 0.1));
  // Seed scans ignore the interval, so they stay one keyframe.
  for (int k = 0; k < 5; ++k) {
    EXPECT_FALSE(builder
                     .onScan(k * kStep, poseAt(0.0), onePoint(1, 0, 0), nullptr,
                             /*seed_scan=*/true)
                     .has_value());
  }
  const auto origin = builder.finish(CloseReason::Seed);
  ASSERT_TRUE(origin.has_value());
  EXPECT_EQ(origin->close_reason, CloseReason::Seed);
  EXPECT_EQ(origin->scan_count, 5U);
  EXPECT_EQ(origin->last_stamp_ns, 4 * kStep);

  builder.onScan(5 * kStep, poseAt(0.0), onePoint(1, 0, 0));
  const auto last = builder.finish(); // the default
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->close_reason, CloseReason::Finish);
  EXPECT_EQ(last->stamp_ns, 5 * kStep);
  EXPECT_EQ(last->last_stamp_ns, 5 * kStep);
}

TEST(CloseReason, NamesAreTheLoggedWords) {
  EXPECT_STREQ(toString(CloseReason::Seed), "seed");
  EXPECT_STREQ(toString(CloseReason::Distance), "distance");
  EXPECT_STREQ(toString(CloseReason::Angle), "angle");
  EXPECT_STREQ(toString(CloseReason::Interval), "interval");
  EXPECT_STREQ(toString(CloseReason::Finish), "finish");
}

} // namespace
