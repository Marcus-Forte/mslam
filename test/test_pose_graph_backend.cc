#include "SyntheticScene.hh"

#include "mslam/slam/PointCloudIO.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/posegraph/LoopCloser.hh"
#include "mslam/slam/posegraph/PoseGraphBackend.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <array>
#include <filesystem>
#include <memory>
#include <set>

namespace {

using mslam::PointCloud;
using mslam::PoseGraph;

std::shared_ptr<spdlog::logger> silentLogger() {
  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);
  return logger;
}

Eigen::Affine3d pose(double x, double y, double z, double rx, double ry,
                     double rz) {
  return mslam::toAffine(x, y, z, rx, ry, rz);
}

// ---------------------------------------------------------------------------
// LoopCloser on a hand-made graph: keyframe 0 and the last one are close, the
// ones between are far away (and empty) so that only 0 is a candidate.

PoseGraph graphWithLoopCandidate(const PointCloud &first,
                                 const PointCloud &last,
                                 const Eigen::Affine3d &last_pose) {
  PoseGraph graph;
  constexpr int kKeyframes = 11; // ids 0..10, separation 10
  for (int id = 0; id < kKeyframes; ++id) {
    mslam::Keyframe kf;
    kf.id = static_cast<std::size_t>(id);
    kf.pose = id == 0                ? Eigen::Affine3d::Identity()
              : id == kKeyframes - 1 ? last_pose
                                     : pose(20.0 + id, 0.0, 0.0, 0.0, 0.0, 0.0);
    kf.pose_odom = kf.pose;
    kf.mini_map = std::make_shared<const PointCloud>(PointCloud{});
    kf.view = std::make_shared<const PointCloud>(id == 0 ? first
                                                 : id == kKeyframes - 1
                                                     ? last
                                                     : PointCloud{});
    graph.keyframes.push_back(kf);
  }
  return graph;
}

mslam::LoopCloser makeCloser(unsigned separation = 10) {
  mslam::PoseGraphParameters parameters;
  parameters.loop_min_separation = separation;
  return mslam::LoopCloser(parameters, silentLogger());
}

TEST(LoopCloser, AcceptsATrueOverlapAndMeasuresTheRelativePose) {
  const std::filesystem::path dir =
      std::filesystem::path(MSLAM_SOURCE_DIR) / "test/data/registration";
  const PointCloud a = mslam::readPlyPointCloud(dir / "a.ply");
  const PointCloud b = mslam::readPlyPointCloud(dir / "b.ply");
  // b.ply's frame sits at this pose in a.ply's frame (poses.jsonc).
  const Eigen::Affine3d truth = pose(0.10, -0.06, 0.04, 0.01, -0.015, 0.03);
  // The graph's estimate is a few cm and 0.02 rad off.
  const Eigen::Affine3d guess = pose(0.13, -0.04, 0.03, 0.02, -0.005, 0.05);

  const PoseGraph graph = graphWithLoopCandidate(a, b, guess);
  const auto result = makeCloser().close(graph, 10);

  ASSERT_EQ(result.attempts.size(), 1U);
  const auto &attempt = result.attempts[0];
  EXPECT_EQ(attempt.candidate, 0U);
  EXPECT_TRUE(attempt.accepted);
  EXPECT_GE(attempt.matched_fraction, 0.6);
  EXPECT_LE(attempt.rms_m, 0.05);
  ASSERT_EQ(result.edges.size(), 1U);
  const auto &edge = result.edges[0];
  EXPECT_EQ(edge.kind, mslam::EdgeKind::Loop);
  EXPECT_TRUE(edge.robust);
  EXPECT_EQ(edge.from, 0U);
  EXPECT_EQ(edge.to, 10U);
  EXPECT_LT((edge.measured.translation() - truth.translation()).norm(), 2e-3);
  const Eigen::Matrix3d relative =
      truth.linear().transpose() * edge.measured.linear();
  EXPECT_LT(Eigen::AngleAxisd(relative).angle(), 1e-3);
}

TEST(LoopCloser, RejectsACloudThatDoesNotMatchTheCandidate) {
  const std::filesystem::path dir =
      std::filesystem::path(MSLAM_SOURCE_DIR) / "test/data/registration";
  const PointCloud a = mslam::readPlyPointCloud(dir / "a.ply");
  const PointCloud c = mslam::readPlyPointCloud(dir / "c.ply");
  // c.ply truly sits 1.7 m / 0.5 rad away; the graph believes it is at a's
  // pose, so the verification starts far outside the gate.
  const PoseGraph graph =
      graphWithLoopCandidate(a, c, Eigen::Affine3d::Identity());
  const auto result = makeCloser().close(graph, 10);

  ASSERT_EQ(result.attempts.size(), 1U);
  EXPECT_FALSE(result.attempts[0].accepted);
  EXPECT_TRUE(result.edges.empty());
}

TEST(LoopCloser, RejectsANonOverlappingScene) {
  const PointCloud room = test::sampleRoom(200.0F, 1, 2.5F);
  PointCloud elsewhere = room;
  // The same kind of surfaces, but 40 m away from where the graph says.
  mslam::transformCloud(pose(40.0, 0.0, 0.0, 0.0, 0.0, 0.0), elsewhere);
  const PoseGraph graph = graphWithLoopCandidate(
      room, elsewhere, pose(0.2, 0.0, 0.0, 0.0, 0.0, 0.0));
  const auto result = makeCloser().close(graph, 10);
  ASSERT_EQ(result.attempts.size(), 1U);
  EXPECT_FALSE(result.attempts[0].accepted);
  EXPECT_LT(result.attempts[0].matched_fraction, 0.6);
}

TEST(LoopCloser, SkipsKeyframesCloserThanTheMinimumSeparation) {
  const PointCloud room = test::sampleRoom(200.0F, 1, 2.5F);
  const PoseGraph graph =
      graphWithLoopCandidate(room, room, pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  // Keyframe 10 with a separation of 11: no keyframe is old enough.
  EXPECT_TRUE(makeCloser(11).close(graph, 10).attempts.empty());
}

// ---------------------------------------------------------------------------
// The whole back end on a synthetic circular drive through a static room.

constexpr int kScans = 80;
constexpr std::int64_t kScanNs = 48'000'000;

Eigen::Affine3d truePose(int k) {
  const double theta = 0.1 * k; // 0.1 m per scan on a circle of radius 1 m
  return pose(std::cos(theta), std::sin(theta), 0.0, 0.0, 0.0,
              theta + M_PI / 2.0);
}

// Odometry that drifts steadily: 1 mm and 0.3 mrad per scan.
Eigen::Affine3d odomPose(int k) {
  return pose(0.001 * k, 0.0, 0.0, 0.0, 0.0, 0.0003 * k) * truePose(k);
}

PointCloud odomScan(int k) {
  PointCloud cloud =
      test::sampleRoom(100.0F, 100 + static_cast<unsigned>(k), 2.5F);
  // The room is static; the sensor sees it from truePose(k) and the front end
  // places it with its own (drifting) pose.
  mslam::transformCloud(truePose(k).inverse(), cloud);
  mslam::transformCloud(odomPose(k), cloud);
  return cloud;
}

mslam::PoseGraphParameters driveParameters(bool synchronous) {
  mslam::PoseGraphParameters p;
  p.keyframe_distance = 0.5;
  p.keyframe_angle = 3.0;
  p.loop_search_radius = 1.5;
  p.loop_min_separation = 5;
  p.loop_submap_keyframes = 1;
  p.synchronous = synchronous;
  return p;
}

std::shared_ptr<const mslam::PoseGraphResult>
drive(bool synchronous, std::vector<mslam::PointCloud> *newly_owned = nullptr) {
  mslam::PoseGraphBackend backend(driveParameters(synchronous), silentLogger());
  for (int k = 0; k < kScans; ++k) {
    auto fresh = backend.onScan(k * kScanNs, odomPose(k), odomScan(k));
    if (newly_owned != nullptr) {
      newly_owned->push_back(std::move(fresh));
    }
  }
  backend.flush();
  return backend.latestResult();
}

TEST(PoseGraphBackend, ClosesALoopAndCorrectsTheDrift) {
  const auto result = drive(true);
  ASSERT_GE(result->poses.size(), 8U);
  EXPECT_GE(result->stats.loop_edges, 1U);
  EXPECT_GE(result->stats.candidates_accepted, 1U);
  EXPECT_GE(result->stats.optimizations, 1U);
  EXPECT_EQ(result->stats.odometry_edges + 1, result->stats.keyframes);
  EXPECT_EQ(result->map_clouds.size(), result->poses.size());
  EXPECT_GT(result->map_version, 0U);
  EXPECT_GE(result->graph_version, result->stats.keyframes);

  // Keyframe 0 is the gauge; the odometry pose of keyframe k is its first
  // scan's, and the truth is the scan's true pose.
  EXPECT_TRUE(result->poses[0].matrix() == odomPose(0).matrix());
  const std::size_t last = result->poses.size() - 1;
  const int scan = static_cast<int>(result->stamps_ns[last] / kScanNs);
  const double odom_error =
      (odomPose(scan).translation() - truePose(scan).translation()).norm();
  const double graph_error =
      (result->poses[last].translation() - truePose(scan).translation()).norm();
  EXPECT_GT(odom_error, 0.04);
  EXPECT_LT(graph_error, odom_error);

  // T_map_odom maps the newest keyframe's odom pose to its graph pose.
  const Eigen::Affine3d corrected = result->T_map_odom * odomPose(scan);
  EXPECT_LT(
      (corrected.translation() - result->poses[last].translation()).norm(),
      1e-9);
}

TEST(PoseGraphBackend, SynchronousAndWorkerGiveTheSameGraph) {
  std::vector<mslam::PointCloud> sync_owned;
  std::vector<mslam::PointCloud> worker_owned;
  const auto sync = drive(true, &sync_owned);
  const auto worker = drive(false, &worker_owned);

  // Ownership is decided on the caller thread in both modes: the per-scan
  // newly owned points and the sealed owned clouds are identical, point for
  // point.
  ASSERT_EQ(sync_owned.size(), worker_owned.size());
  for (std::size_t k = 0; k < sync_owned.size(); ++k) {
    ASSERT_EQ(sync_owned[k].size(), worker_owned[k].size()) << k;
    for (std::size_t i = 0; i < sync_owned[k].size(); ++i) {
      EXPECT_EQ(sync_owned[k][i].x, worker_owned[k][i].x);
      EXPECT_EQ(sync_owned[k][i].y, worker_owned[k][i].y);
      EXPECT_EQ(sync_owned[k][i].z, worker_owned[k][i].z);
    }
  }
  ASSERT_EQ(sync->mini_maps.size(), worker->mini_maps.size());
  for (std::size_t k = 0; k < sync->mini_maps.size(); ++k) {
    const auto &a = *sync->mini_maps[k];
    const auto &b = *worker->mini_maps[k];
    ASSERT_EQ(a.size(), b.size()) << k;
    for (std::size_t i = 0; i < a.size(); ++i) {
      EXPECT_EQ(a[i].x, b[i].x);
      EXPECT_EQ(a[i].y, b[i].y);
      EXPECT_EQ(a[i].z, b[i].z);
    }
  }
  EXPECT_EQ(sync->stats.view_points, worker->stats.view_points);

  ASSERT_EQ(sync->poses.size(), worker->poses.size());
  for (std::size_t k = 0; k < sync->poses.size(); ++k) {
    EXPECT_TRUE(sync->poses[k].matrix() == worker->poses[k].matrix()) << k;
    EXPECT_EQ(sync->stamps_ns[k], worker->stamps_ns[k]);
    ASSERT_EQ(sync->map_clouds[k]->size(), worker->map_clouds[k]->size());
  }
  ASSERT_EQ(sync->edges.size(), worker->edges.size());
  for (std::size_t e = 0; e < sync->edges.size(); ++e) {
    EXPECT_EQ(sync->edges[e].from, worker->edges[e].from);
    EXPECT_EQ(sync->edges[e].to, worker->edges[e].to);
    EXPECT_TRUE(sync->edges[e].measured.matrix() ==
                worker->edges[e].measured.matrix());
  }
  EXPECT_EQ(sync->graph_version, worker->graph_version);
  EXPECT_EQ(sync->map_version, worker->map_version);
  EXPECT_EQ(sync->stats.loop_edges, worker->stats.loop_edges);
  EXPECT_EQ(sync->stats.candidates_rejected, worker->stats.candidates_rejected);
  EXPECT_TRUE(sync->T_map_odom.matrix() == worker->T_map_odom.matrix());
}

TEST(PoseGraphBackend, TheOwnedMapHasEachOdomVoxelOnceAndViewsOverlap) {
  const auto result = drive(true);
  ASSERT_GE(result->poses.size(), 8U);

  // Each keyframe's owned cloud, put back into the odom frame by the
  // keyframe's odometry pose, never repeats a voxel of another keyframe's.
  std::set<std::array<int, 3>> seen;
  std::size_t owned_points = 0;
  for (std::size_t k = 0; k < result->mini_maps.size(); ++k) {
    const int scan = static_cast<int>(result->stamps_ns[k] / kScanNs);
    const Eigen::Affine3d odom = odomPose(scan);
    for (const auto &point : *result->mini_maps[k]) {
      const Eigen::Vector3d p =
          odom * Eigen::Vector3d(point.x, point.y, point.z);
      const std::array<int, 3> voxel = {
          static_cast<int>(std::floor(p.x() / 0.05)),
          static_cast<int>(std::floor(p.y() / 0.05)),
          static_cast<int>(std::floor(p.z() / 0.05))};
      EXPECT_TRUE(seen.insert(voxel).second) << "keyframe " << k;
      ++owned_points;
    }
  }
  // The views, which hold everything each keyframe saw, overlap a lot: the
  // room is seen from every keyframe.
  EXPECT_EQ(owned_points, seen.size());
  EXPECT_GT(result->stats.view_points, 3 * owned_points);
}

TEST(PoseGraphBackend, SealOpenKeyframeMakesTheSeedScansTheOrigin) {
  mslam::PoseGraphBackend backend(driveParameters(true), silentLogger());
  for (int k = 0; k < 5; ++k) {
    backend.onScan(
        k * kScanNs, Eigen::Affine3d::Identity(),
        test::sampleRoom(100.0F, 7 + static_cast<unsigned>(k), 2.5F));
  }
  EXPECT_TRUE(backend.latestResult()->poses.empty()); // nothing sealed yet
  backend.sealOpenKeyframe();
  const auto origin = backend.latestResult();
  ASSERT_EQ(origin->poses.size(), 1U); // synchronous: processed at once
  EXPECT_TRUE(origin->poses[0].matrix().isIdentity(0.0));
  EXPECT_EQ(backend.sealedKeyframes(), 1U);
  EXPECT_FALSE(backend.hasOpenKeyframe());

  // The next scan opens keyframe 1 and seals nothing.
  backend.onScan(5 * kScanNs, odomPose(0), odomScan(0));
  EXPECT_EQ(backend.latestResult()->poses.size(), 1U);
  EXPECT_TRUE(backend.hasOpenKeyframe());
}

TEST(PoseGraphBackend, ResetClearsTheGraphAndKeepsTheVersionGrowing) {
  mslam::PoseGraphBackend backend(driveParameters(false), silentLogger());
  for (int k = 0; k < 20; ++k) {
    backend.onScan(k * kScanNs, odomPose(k), odomScan(k));
  }
  backend.flush();
  const auto before = backend.latestResult();
  ASSERT_GT(before->poses.size(), 0U);

  backend.reset();
  const auto after = backend.latestResult();
  EXPECT_TRUE(after->poses.empty());
  EXPECT_TRUE(after->T_map_odom.matrix().isIdentity());
  EXPECT_GT(after->graph_version, before->graph_version);
  EXPECT_FALSE(backend.hasOpenKeyframe());
}

TEST(PoseGraphBackend, DestructorStopsTheWorkerWithWorkQueued) {
  mslam::PoseGraphBackend backend(driveParameters(false), silentLogger());
  for (int k = 0; k < 40; ++k) {
    backend.onScan(k * kScanNs, odomPose(k), odomScan(k));
  }
  // No flush: destruction must not hang or crash.
}

} // namespace
