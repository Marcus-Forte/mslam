#include "SyntheticScene.hh"

#include "mslam/slam/ScanSequenceRegistration.hh"
#include "mslam/slam/Transform.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <optional>
#include <vector>

namespace {

std::shared_ptr<spdlog::logger> silentLogger() {
  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);
  return logger;
}

mslam::Pose3D makePose(double x, double y, double z, double roll, double pitch,
                       double yaw) {
  mslam::Pose3D pose;
  pose << x, y, z, roll, pitch, yaw;
  return pose;
}

Eigen::Affine3d toAffine(const mslam::Pose3D &pose) {
  return mslam::toAffine(pose(0), pose(1), pose(2), pose(3), pose(4), pose(5));
}

// The room points as a sensor at @p pose sees them: pose * seen = room.
mslam::PointCloud seenFrom(mslam::PointCloud room, const mslam::Pose3D &pose) {
  mslam::transformCloud(toAffine(pose).inverse(), room);
  return room;
}

// The points of the room that lie on the floor, wall x or wall y.
mslam::PointCloud surfacesOf(const mslam::PointCloud &room, bool floor,
                             bool wall_x, bool wall_y) {
  mslam::PointCloud kept;
  for (const auto &point : room) {
    if ((floor && point.z == -1.0F) || (wall_x && point.x == 3.0F) ||
        (wall_y && point.y == 3.0F)) {
      kept.push_back(point);
    }
  }
  return kept;
}

void expectPose(const mslam::SlamState &state, const mslam::Pose3D &truth) {
  EXPECT_NEAR(state.position.x(), truth(0), 5e-3);
  EXPECT_NEAR(state.position.y(), truth(1), 5e-3);
  EXPECT_NEAR(state.position.z(), truth(2), 5e-3);
  EXPECT_NEAR(state.rotation.x(), truth(3), 2e-3);
  EXPECT_NEAR(state.rotation.y(), truth(4), 2e-3);
  EXPECT_NEAR(state.rotation.z(), truth(5), 2e-3);
}

mslam::ScanSequenceRegistration makeSequence() {
  return mslam::ScanSequenceRegistration(mslam::ScanSequenceParameters{},
                                         silentLogger());
}

// Three samplings of the room. The reference is dense and wide so that the
// displaced, smaller later clouds stay inside it.
mslam::PointCloud reference() { return test::sampleRoom(400.0F, 1, 3.0F); }
mslam::PointCloud sampling(unsigned seed) {
  return test::sampleRoom(150.0F, seed, 2.5F);
}

TEST(ScanSequenceRegistration, RecoversDisplacedSamplingsOfTheSameRoom) {
  const auto truth1 = makePose(0.05, -0.03, 0.02, 0.01, -0.02, 0.03);
  const auto truth2 = makePose(0.02, -0.01, 0.03, 0.005, -0.01, 0.045);
  auto sequence = makeSequence();

  const auto first = sequence.add(reference());
  const auto second = sequence.add(seenFrom(sampling(2), truth1));
  const auto third = sequence.add(seenFrom(sampling(3), truth2));

  EXPECT_FALSE(first.registered);
  EXPECT_EQ(first.pose.position, Eigen::Vector3d::Zero());
  EXPECT_EQ(first.pose.rotation, Eigen::Vector3d::Zero());
  EXPECT_FALSE(first.prior_measured);
  EXPECT_GT(first.input_points, 0U);
  EXPECT_GT(first.used_points, 0U);
  EXPECT_LE(first.used_points, first.input_points);

  ASSERT_TRUE(second.registered);
  ASSERT_TRUE(third.registered);
  EXPECT_TRUE(second.first_round.valid);
  EXPECT_TRUE(third.first_round.valid);
  expectPose(second.pose, truth1);
  expectPose(third.pose, truth2);
  EXPECT_EQ(sequence.currentPose().position, third.pose.position);

  // A good fit: most points match and the residual is millimetres.
  EXPECT_GT(third.matched_points, third.used_points / 2);
  EXPECT_LT(third.rms_m, 0.01);
}

TEST(ScanSequenceRegistration,
     WithoutMeasuredPosesEachStepStartsFromThePrevious) {
  auto sequence = makeSequence();
  const auto first = sequence.add(reference());
  const auto second = sequence.add(
      seenFrom(sampling(2), makePose(0.05, -0.03, 0.02, 0.01, -0.02, 0.03)));
  const auto third = sequence.add(
      seenFrom(sampling(3), makePose(0.02, -0.01, 0.03, 0.005, -0.01, 0.045)));

  EXPECT_FALSE(second.prior_measured);
  EXPECT_FALSE(third.prior_measured);
  // Exactly the stored values, not merely close ones.
  EXPECT_EQ(second.prior.position, first.pose.position);
  EXPECT_EQ(second.prior.rotation, first.pose.rotation);
  EXPECT_EQ(third.prior.position, second.pose.position);
  EXPECT_EQ(third.prior.rotation, second.pose.rotation);
}

TEST(ScanSequenceRegistration,
     MeasuredPoseRecoversADisplacementBeyondICPRange) {
  // About 1.5 m and 0.5 rad: far outside the 0.3 m correspondence gate.
  const auto truth1 = makePose(1.5, -0.4, 0.05, 0.0, 0.0, 0.5);
  const auto measured1 = makePose(1.53, -0.42, 0.06, 0.005, -0.005, 0.51);
  const auto truth2 = makePose(1.52, -0.43, 0.04, 0.005, -0.01, 0.52);
  auto sequence = makeSequence();

  sequence.add(reference());
  const auto second =
      sequence.add(seenFrom(sampling(2), truth1), std::optional(measured1));
  EXPECT_TRUE(second.prior_measured);
  EXPECT_EQ(second.prior.position, measured1.head<3>().transpose());
  EXPECT_EQ(second.prior.rotation, measured1.tail<3>().transpose());
  ASSERT_TRUE(second.first_round.valid);
  expectPose(second.pose, truth1);

  // Without a measured pose the next cloud starts from the result above.
  const auto third = sequence.add(seenFrom(sampling(3), truth2));
  EXPECT_FALSE(third.prior_measured);
  EXPECT_EQ(third.prior.position, second.pose.position);
  EXPECT_EQ(third.prior.rotation, second.pose.rotation);
  expectPose(third.pose, truth2);
}

TEST(ScanSequenceRegistration, MeasuredPoseOnTheFirstCloudDefinesTheFrame) {
  // The first cloud sits at `frame` in the room (the common frame); the later
  // cloud is registered into that frame.
  const auto frame = makePose(0.5, -0.2, 0.1, 0.0, 0.0, 0.3);
  const auto truth1 = makePose(0.55, -0.23, 0.12, 0.01, -0.01, 0.32);
  auto sequence = makeSequence();

  const auto first =
      sequence.add(seenFrom(reference(), frame), std::optional(frame));
  EXPECT_FALSE(first.registered);
  EXPECT_TRUE(first.prior_measured);
  EXPECT_EQ(first.pose.position, frame.head<3>().transpose());
  EXPECT_EQ(first.pose.rotation, frame.tail<3>().transpose());
  EXPECT_EQ(sequence.currentPose().position, first.pose.position);

  const auto second = sequence.add(seenFrom(sampling(2), truth1));
  EXPECT_EQ(second.prior.position, first.pose.position);
  ASSERT_TRUE(second.first_round.valid);
  expectPose(second.pose, truth1);
}

TEST(ScanSequenceRegistration, TheMapAccumulatesAllCloudsRegisteredSoFar) {
  // `a` sees the floor and wall x, `b` the floor and wall y; neither fixes
  // the other's missing wall. Only the full room `c` is constrained in all six
  // degrees of freedom, and only by both earlier clouds together: its x needs
  // a's wall x, its y and yaw need b's wall y, and it starts about 0.06 m off
  // in x and y, so a target missing either wall could not recover it.
  // `a` and `b` stop at extent 2.6 so that their walls stay more than the
  // correspondence gate (0.3 m) apart near the corner (3, 3): within the gate,
  // b's wall y would match a's wall x and drag b away from where it was put.
  const auto room_a =
      surfacesOf(test::sampleRoom(400.0F, 1, 2.6F), true, true, false);
  const auto room_b =
      surfacesOf(test::sampleRoom(400.0F, 2, 2.6F), true, false, true);
  const auto pose_b = makePose(0.02, -0.01, 0.0, 0.0, 0.0, 0.005);
  const auto pose_c = makePose(0.08, -0.06, 0.03, 0.0, 0.0, 0.03);
  auto sequence = makeSequence();

  sequence.add(room_a);
  const std::size_t voxels_a = sequence.mapVoxels();
  // b against a alone is constrained by the floor only, so its exact pose is
  // given and it must not move in x, y or yaw.
  const auto step_b =
      sequence.add(seenFrom(room_b, pose_b), std::optional(pose_b));
  const std::size_t voxels_b = sequence.mapVoxels();
  const auto step_c = sequence.add(seenFrom(sampling(3), pose_c));
  const std::size_t voxels_c = sequence.mapVoxels();

  EXPECT_GT(voxels_a, 0U);
  EXPECT_GT(voxels_b, voxels_a);
  EXPECT_GE(voxels_c, voxels_b);
  EXPECT_TRUE(step_b.prior_measured);
  expectPose(step_b.pose, pose_b);
  ASSERT_TRUE(step_c.first_round.valid);
  expectPose(step_c.pose, pose_c);
}

TEST(ScanSequenceRegistration, ACloudWithoutOverlapKeepsItsPrior) {
  auto sequence = makeSequence();
  sequence.add(reference());

  mslam::PointCloud far_away = sampling(2);
  mslam::transformCloud(
      Eigen::Affine3d(Eigen::Translation3d(100.0, 100.0, 100.0)), far_away);
  const auto step = sequence.add(far_away);

  EXPECT_TRUE(step.registered);
  EXPECT_FALSE(step.first_round.valid);
  EXPECT_EQ(step.matched_points, 0U);
  EXPECT_NEAR((step.pose.position - step.prior.position).norm(), 0.0, 1e-9);
  EXPECT_NEAR((step.pose.rotation - step.prior.rotation).norm(), 0.0, 1e-9);
}

TEST(ScanSequenceRegistration, SearchRadiusFollowsTheCorrespondenceGate) {
  const auto radius = [](float gate, float resolution) {
    mslam::ScanSequenceParameters parameters;
    parameters.map_resolution = resolution;
    parameters.icp.max_correspondence_distance = gate;
    return mslam::ScanSequenceRegistration(parameters, silentLogger())
        .searchRadiusVoxels();
  };
  EXPECT_EQ(radius(0.3F, 0.1F), 3);  // float 0.3 / 0.1 is just above 3
  EXPECT_EQ(radius(0.05F, 0.1F), 1); // at least one ring
  EXPECT_EQ(radius(0.25F, 0.1F), 3); // rounded up
  EXPECT_EQ(radius(5.0F, 0.1F), mslam::VoxelHashMap::kMaxAdjacentVoxels);
}

} // namespace
