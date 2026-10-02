
#include "ConsoleLogger.hh"
#include "config/JsonConfig.hh"

#include "map/VoxelHashMap.hh"
#include "msensor/config/config.hh"
#include "msensor/lidar/mid360.hh"
#include "sensors_remote_client.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/Slam.hh"
#include "slam/SlamServer.hh"
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <stdexcept>

const unsigned int g_default_playback_delay_ms = 10;

namespace {

void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name
            << " [-c config.json] [-d delay_ms] [-f recording.pbscan] "
               "[-h]\n"
            << "  -c <file>  Load SLAM configuration from JSON\n"
            << "  -d <ms>    Delay between playback entries when using -f\n"
            << "  -f <file>  Replay a recorded scan file instead of connecting "
               "remotely\n"
            << "  -h         Show this help message\n";
}
} // namespace

int main(int argc, char **argv) {
  int opt;
  mslam::SlamConfiguration config;
  std::filesystem::path slam_config_path;
  std::string slam_play_file = "";
  unsigned int playback_delay_ms = g_default_playback_delay_ms;
  while ((opt = getopt(argc, argv, "c:d:f:h")) != -1) {
    switch (opt) {
    case 'c': {
      std::cout << "Using config: " << optarg << std::endl;
      slam_config_path = optarg;
      mslam::JsonConfig json_config(optarg);
      json_config.load();
      config = json_config.getConfig();
      break;
    }
    case 'd':
      playback_delay_ms = static_cast<unsigned int>(std::stoul(optarg));
      break;
    case 'f':
      std::cout << "Using recorded sensor playback with: " << optarg
                << std::endl;
      slam_play_file = optarg;
      break;
    case 'h':
      printUsage(argv[0]);
      return 0;

    case '?':
      printUsage(argv[0]);
      return 1;
      break;
    }
  }

  if (!slam_play_file.empty()) {
    config.remote_scanner = "local";
  }

  std::cout << config << std::endl;

  // Create logger.
  const auto logger = std::make_shared<ConsoleLogger>();
  logger->setLevel(config.log_level);

  // Create Map interface.
  std::shared_ptr<mslam::IMap> map;
  auto voxel_map = std::make_shared<mslam::VoxelHashMap>(
      config.map_parameters.resolution,
      config.map_parameters.max_points_per_voxel);
  voxel_map->setNumAdjacentVoxelSearch(1); /// \todo add configurable?
  map = std::move(voxel_map);

  mslam::SlamServer slam_server(logger, map);
  slam_server.start();

  // Create sensor readers.
  std::shared_ptr<msensor::ILidar> lidar_sensor;
  std::shared_ptr<msensor::IImu> imu_sensor;
  std::shared_ptr<mslam::RecordingSensorPlayer> playback_player;

  if (!slam_play_file.empty()) {
    playback_player = std::make_shared<mslam::RecordingSensorPlayer>(
        slam_play_file, logger, config.with_imu, config.with_lidar,
        playback_delay_ms);
    playback_player->init();
    playback_player->startSampling();
    lidar_sensor = std::dynamic_pointer_cast<msensor::ILidar>(playback_player);
    imu_sensor = std::dynamic_pointer_cast<msensor::IImu>(playback_player);
    logger->log(ILog::Level::INFO,
                "Initialized recording playback player with file: {}",
                slam_play_file);
  } else if (config.remote_scanner == "local") {
    auto sensor_config_path =
        slam_config_path.empty()
            ? std::filesystem::path("config/publisher_config.json")
            : slam_config_path.parent_path() / "publisher_config.json";
    if (!std::filesystem::exists(sensor_config_path)) {
      sensor_config_path = msensor::Config::defaultConfigPath();
    }
    auto sensor_config = msensor::Config::fromFile(sensor_config_path);
    if (!sensor_config.mid360.enable) {
      throw std::runtime_error(
          "Local mode requires mid360.enable in the msensor publisher config");
    }
    if (sensor_config.mid360.config.empty()) {
      throw std::runtime_error(
          "Local mode requires mid360.config in the msensor publisher config");
    }

    std::filesystem::path mid360_config_path(sensor_config.mid360.config);
    if (mid360_config_path.is_relative()) {
      mid360_config_path =
          sensor_config_path.parent_path() / mid360_config_path;
    }
    if (!std::filesystem::exists(mid360_config_path)) {
      throw std::runtime_error("Mid360 config file does not exist: " +
                               mid360_config_path.string());
    }

    auto mid360 = std::make_shared<msensor::Mid360>(
        mid360_config_path.string(),
        sensor_config.mid360.accumulate_scan_count);
    mid360->init();
    mid360->setMode(msensor::Mid360::Mode::Normal);
    mid360->setScanPattern(msensor::Mid360::ScanPattern::NonRepetitive);
    mid360->startSampling();
    lidar_sensor = mid360;
    imu_sensor = mid360;
    logger->log(ILog::Level::INFO, "Initialized local Mid360 using config: {}",
                mid360_config_path.string());

  } else {
    auto remote = std::make_shared<SensorsRemoteClient>(config.remote_scanner);
    remote->init();
    remote->start();
    lidar_sensor = remote;
    imu_sensor = remote;
  }

  mslam::Slam slam(logger, config, map);
  slam_server.setSlam(&slam);

  slam.run(lidar_sensor, imu_sensor, slam_server, playback_player);

  return 0;
}