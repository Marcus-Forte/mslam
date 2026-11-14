#include "SyntheticScene.hh"

#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/CorrespondenceFinder.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/IRegistration.hh"
#include "mslam/slam/registration/IcpWorkspace.hh"
#include "mslam/slam/registration/PointToPlaneRegistration.hh"

#include <gtest/gtest.h>
#include <random>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <sstream>

namespace {

std::shared_ptr<spdlog::logger> silentLogger() {
  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);
  return logger;
}

// The scan is the room seen from a pose displaced by `truth`: registering it
// against the room map must recover that pose. The motions are small because
// the map only looks one voxel (0.1 m) around each point for a correspondence.
class PointToPlaneRecovery
    : public ::testing::TestWithParam<
          std::tuple<double, double, double, double, double, double>> {};

TEST_P(PointToPlaneRecovery, RecoversKnownPose) {
  const auto [x, y, z, roll, pitch, yaw] = GetParam();
  const Eigen::Affine3d truth = mslam::toAffine(x, y, z, roll, pitch, yaw);

  mslam::VoxelHashMap map(0.1F, 5);
  map.addScan(test::sampleRoom(400.0F, 1));

  // scan = truth^-1 * (room sampled at another phase): truth * scan lies on
  // the map surfaces.
  mslam::PointCloud scan = test::sampleRoom(150.0F, 2, 2.5F);
  mslam::transformCloud(truth.inverse(), scan);

  mslam::PointToPlaneRegistration registration(
      10, 5, 0.5F, silentLogger(),
      std::make_shared<mslam::CorrespondenceFinder>());
  const auto state = registration.Align(mslam::SlamState{}, map, scan);

  EXPECT_NEAR(state.position.x(), x, 5e-3);
  EXPECT_NEAR(state.position.y(), y, 5e-3);
  EXPECT_NEAR(state.position.z(), z, 5e-3);
  EXPECT_NEAR(state.rotation.x(), roll, 2e-3);
  EXPECT_NEAR(state.rotation.y(), pitch, 2e-3);
  EXPECT_NEAR(state.rotation.z(), yaw, 2e-3);
}

INSTANTIATE_TEST_SUITE_P(
    SmallMotions, PointToPlaneRecovery,
    ::testing::Values(std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                      std::make_tuple(0.05, -0.03, 0.02, 0.0, 0.0, 0.0),
                      std::make_tuple(0.0, 0.0, 0.0, 0.01, -0.02, 0.03),
                      std::make_tuple(0.04, 0.03, -0.02, 0.01, 0.008, -0.02)));

TEST(PointToPlaneRegistration, WithoutOverlapReturnsThePriorPose) {
  mslam::VoxelHashMap map(0.1F, 5);
  map.addScan(test::sampleRoom(400.0F, 1));

  mslam::PointCloud far_scan;
  far_scan.emplace_back(100.0F, 100.0F, 100.0F, 1.0F);

  mslam::PointToPlaneRegistration registration(
      3, 3, 0.3F, silentLogger(),
      std::make_shared<mslam::CorrespondenceFinder>());
  mslam::SlamState prior;
  prior.position = {0.1, 0.2, 0.3};
  prior.rotation = {0.01, 0.02, 0.03};
  const auto state = registration.Align(prior, map, far_scan);
  EXPECT_NEAR((state.position - prior.position).norm(), 0.0, 1e-12);
  EXPECT_NEAR((state.rotation - prior.rotation).norm(), 0.0, 1e-12);
}

TEST(PointToPlaneRegistration, IsReusableAcrossAlignCalls) {
  mslam::VoxelHashMap map(0.1F, 5);
  map.addScan(test::sampleRoom(400.0F, 1));
  mslam::PointToPlaneRegistration registration(
      10, 5, 0.5F, silentLogger(),
      std::make_shared<mslam::CorrespondenceFinder>());

  const Eigen::Affine3d first = mslam::toAffine(0.04, 0.0, 0.0, 0, 0, 0.02);
  mslam::PointCloud scan_a = test::sampleRoom(150.0F, 2, 2.5F);
  mslam::transformCloud(first.inverse(), scan_a);
  const auto state_a = registration.Align(mslam::SlamState{}, map, scan_a);

  // A second registration against a changed map must not see cached normals
  // from the first one.
  map.clear();
  mslam::PointCloud shifted_room = test::sampleRoom(400.0F, 1);
  mslam::transformCloud(mslam::toAffine(0.0, 0.0, 0.0, 0, 0, 0.0),
                        shifted_room);
  map.addScan(shifted_room);
  mslam::PointCloud scan_b = test::sampleRoom(150.0F, 2, 2.5F);
  const Eigen::Affine3d second = mslam::toAffine(-0.03, 0.02, 0.0, 0, 0, -0.01);
  mslam::transformCloud(second.inverse(), scan_b);
  const auto state_b = registration.Align(mslam::SlamState{}, map, scan_b);

  EXPECT_NEAR(state_a.position.x(), 0.04, 5e-3);
  EXPECT_NEAR(state_b.position.x(), -0.03, 5e-3);
  EXPECT_NEAR(state_b.rotation.z(), -0.01, 2e-3);
}

std::shared_ptr<spdlog::logger>
capturingLogger(std::ostringstream &out, spdlog::level::level_enum level) {
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(out);
  auto logger = std::make_shared<spdlog::logger>("registration_test", sink);
  logger->set_level(level);
  return logger;
}

mslam::PointCloud shiftedRoomScan() {
  mslam::PointCloud scan = test::sampleRoom(150.0F, 2, 2.5F);
  mslam::transformCloud(mslam::toAffine(0.03, 0.0, 0.0, 0, 0, 0.0).inverse(),
                        scan);
  return scan;
}

// The optimizer telemetry is only collected when debug logging is on.
TEST(PointToPlaneRegistration, OptimizerObserverIsAttachedOnlyForDebugLogging) {
  mslam::VoxelHashMap map(0.1F, 5);
  map.addScan(test::sampleRoom(400.0F, 1));
  const auto scan = shiftedRoomScan();

  std::ostringstream info_out;
  mslam::PointToPlaneRegistration quiet(
      3, 3, 0.5F, capturingLogger(info_out, spdlog::level::info),
      std::make_shared<mslam::CorrespondenceFinder>());
  quiet.Align(mslam::SlamState{}, map, scan);
  EXPECT_EQ(info_out.str().find("linear system"), std::string::npos);
  EXPECT_TRUE(info_out.str().empty());

  std::ostringstream debug_out;
  mslam::PointToPlaneRegistration verbose(
      3, 3, 0.5F, capturingLogger(debug_out, spdlog::level::debug),
      std::make_shared<mslam::CorrespondenceFinder>());
  verbose.Align(mslam::SlamState{}, map, scan);
  EXPECT_NE(debug_out.str().find("linear system"), std::string::npos);
  EXPECT_NE(debug_out.str().find("Registration Iteration"), std::string::npos);
}

TEST(IcpWorkspace, PlaneConstraintsHoldSourceNormalAndPlaneOffset) {
  // A map that is the plane z = 0.5, sampled densely.
  mslam::VoxelHashMap map(0.1F, 5);
  mslam::PointCloud plane;
  std::mt19937 rng(4);
  std::uniform_real_distribution<float> uniform(-1.0F, 1.0F);
  for (int i = 0; i < 4000; ++i) {
    plane.emplace_back(uniform(rng), uniform(rng), 0.5F, 1.0F);
  }
  map.addScan(plane);

  mslam::PointCloud scan;
  scan.emplace_back(0.2F, 0.1F, 0.52F, 1.0F);
  scan.emplace_back(50.0F, 50.0F, 50.0F, 1.0F); // no correspondence

  mslam::IcpWorkspace workspace(
      mslam::IcpParameters{3, 3, 0.2F}, silentLogger(),
      std::make_shared<mslam::CorrespondenceFinder>());
  workspace.begin(map, scan, Eigen::Affine3d::Identity());
  ASSERT_EQ(workspace.findPlaneConstraints(map), 1U);

  const auto inputs = workspace.planeInputs();
  const auto offsets = workspace.planeOffsets();
  ASSERT_EQ(inputs.size(), 1U);
  ASSERT_EQ(offsets.size(), 1U);
  EXPECT_NEAR(inputs[0][0], 0.2, 1e-6);
  EXPECT_NEAR(inputs[0][1], 0.1, 1e-6);
  EXPECT_NEAR(inputs[0][2], 0.52, 1e-6);
  const Eigen::Vector3d normal = inputs[0].tail<3>();
  EXPECT_NEAR(normal.norm(), 1.0, 1e-9);
  EXPECT_NEAR(std::abs(normal.z()), 1.0, 1e-4);
  // normal . map_point with map_point on z = 0.5 (sign follows the normal).
  EXPECT_NEAR(offsets[0], normal.z() * 0.5, 1e-3);

  // Moving the source moves the constraint's source point; resetting rebuilds
  // it from the scan.
  workspace.moveSource(mslam::toAffine(0.0, 0.0, 0.01, 0.0, 0.0, 0.0));
  EXPECT_NEAR(workspace.source()[0].z, 0.53F, 1e-6F);
  workspace.resetSource(Eigen::Affine3d::Identity());
  EXPECT_NEAR(workspace.source()[0].z, 0.52F, 1e-6F);
}

} // namespace
