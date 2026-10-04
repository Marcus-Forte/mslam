#include "config/JsonConfig.hh"
#include "map/VoxelHashMap.hh"
#include "msensor/recorder/recording_driver.hh"
#include "msensor_server.hh"
#include "sensors_remote_client.hh"
#include "slam/Slam.hh"
#include "slam/SlamServer.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

// A loop-closure / drift check: the recording walks a closed loop, so the final
// position should be back near the origin. Only translation is asserted here.
// This is LiDAR-inertial odometry without loop closure, so heading is not
// expected to return to zero (the living_room_garden run ends ~0.49 rad off).
constexpr double k_max_origin_offset_m = 0.1;

// The gRPC sensor server is reachable on this test-only port. Using a
// non-default port avoids clashing with a real `sensor_publisher`.
constexpr int k_test_port = 50071;

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

  // The SLAM output server is only used by the run loop as a pose/map sink; it
  // is not started, so no port is bound.
  mslam::SlamServer slam_server(logger);

  // Replay the recording as a standalone gRPC sensor server and consume it
  // through the regular remote client, exercising the full production path.
  const std::string address = "127.0.0.1:" + std::to_string(k_test_port);
  // Raw max speed (0) replays the file faster than the gRPC stream can drain,
  // so the server's single-pending-slot policy drops samples and the run
  // diverges. 16x keeps every sample on this host (verified up to 32x; 64x
  // drops) while staying well ahead of real time. Override with
  // MSLAM_REPLAY_SPEED for local experiments.
  double speed = 16.0;
  if (const char *env = std::getenv("MSLAM_REPLAY_SPEED")) {
    speed = std::stod(env);
  }
  auto driver =
      std::make_shared<msensor::RecordingSensorDriver>(recording, speed);
  driver->init();

  SensorsServer sensor_server(nullptr, nullptr, driver, driver, address);
  sensor_server.start();

  auto client = std::make_shared<SensorsRemoteClient>(address);
  client->init();
  client->start();

  // Start the replay only after the client stream and SLAM callbacks are
  // registered, otherwise the first samples would be dropped. The production
  // SLAM loop intentionally stays alive when the sensor stream goes idle;
  // stop this integration run explicitly after the recording is consumed.
  std::thread replay_thread([&driver]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    driver->startSampling();
    while (!driver->isFinished()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    std::raise(SIGINT);
  });

  auto lidar = std::static_pointer_cast<msensor::ILidar>(client);
  auto imu = std::static_pointer_cast<msensor::IImu>(client);
  mslam::Slam slam(logger, config, map);
  slam_server.setSlam(&slam);
  slam.run(lidar, imu, slam_server);

  replay_thread.join();

  ASSERT_TRUE(driver->isFinished())
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

  client->stop();
  driver->stopSampling();
  sensor_server.stop();

  EXPECT_LT(position_error_m, k_max_origin_offset_m)
      << "Final position error is " << position_error_m << " m (pos=["
      << pose[0] << ", " << pose[1] << ", " << pose[2] << "])";
}

} // namespace
