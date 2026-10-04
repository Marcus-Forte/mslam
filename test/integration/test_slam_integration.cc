#include "config/JsonConfig.hh"
#include "map/VoxelHashMap.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/Slam.hh"
#include "slam/SlamServer.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

// A loop-closure / drift check: the recording walks a closed loop, so the final
// position should be back near the origin. Only translation is asserted here.
// This is LiDAR-inertial odometry without loop closure, so heading is not
// expected to return to zero (the living_room_garden run ends ~0.49 rad off).
constexpr double k_max_origin_offset_m = 0.1;

TEST(SlamIntegration, LivingRoomGardenReturnsToOrigin) {
  const std::filesystem::path source_dir{MSLAM_SOURCE_DIR};
  const auto recording = source_dir / "test/data/living_room_garden.pbscan";
  if (!std::filesystem::exists(recording)) {
    GTEST_SKIP() << "Missing recording: " << recording
                 << " (untracked sample data)";
  }

  // Test-scoped config, deliberately independent of config/mslam.jsonc.
  mslam::JsonConfig json_config(source_dir / "test/integration/mslam.jsonc");
  json_config.load();
  const auto config = json_config.getConfig();

  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);

  auto map = std::make_shared<mslam::VoxelHashMap>(
      config.map_parameters.resolution,
      config.map_parameters.max_points_per_voxel);
  map->setNumAdjacentVoxelSearch(1);

  // The gRPC server is only used by the run loop as a pose/map sink; it is not
  // started, so no port is bound.
  mslam::SlamServer slam_server(logger);
  auto playback_player = std::make_shared<mslam::RecordingSensorPlayer>(
      recording, logger, config.with_imu, config.with_lidar, 0);
  playback_player->init();
  playback_player->startSampling();

  mslam::Slam slam(logger, config, map);
  slam_server.setSlam(&slam);

  auto lidar = std::static_pointer_cast<msensor::ILidar>(playback_player);
  auto imu = std::static_pointer_cast<msensor::IImu>(playback_player);
  slam.run(lidar, imu, slam_server, playback_player);

  ASSERT_TRUE(playback_player->isFinished())
      << "Playback did not consume the whole recording";

  const auto pose = slam.getPose();
  const double position_error_m =
      std::sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
  const double rotation_error_rad =
      std::sqrt(pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5]);

  const std::string position_error = std::to_string(position_error_m);
  const std::string rotation_error = std::to_string(rotation_error_rad);
  RecordProperty("final_position_error_m", position_error);
  RecordProperty("final_rotation_error_rad", rotation_error);

  std::cout << "Final pose: pos=[" << pose[0] << ", " << pose[1] << ", "
            << pose[2] << "] rot=[" << pose[3] << ", " << pose[4] << ", "
            << pose[5] << "]\n"
            << "Final error: position=" << position_error_m
            << " m, rotation=" << rotation_error_rad << " rad\n";

  EXPECT_LT(position_error_m, k_max_origin_offset_m)
      << "Final position error is " << position_error_m << " m (pos=["
      << pose[0] << ", " << pose[1] << ", " << pose[2] << "])";
}

} // namespace
