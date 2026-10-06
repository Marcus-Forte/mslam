#include "mslam/config/JsonConfig.hh"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "msensor/config/config.hh"
#include "msensor/lidar/mid360.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamServer.hh"
#include "sensors_remote_client.hh"
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name << " [-c config.json] [-h]\n"
            << "  -c <file>  Load SLAM configuration from JSON\n"
            << "  -h         Show this help message\n";
}
} // namespace

int main(int argc, char **argv) {
  int opt;
  mslam::SlamConfiguration config;
  std::filesystem::path slam_config_path;
  while ((opt = getopt(argc, argv, "c:h")) != -1) {
    switch (opt) {
    case 'c': {
      std::cout << "Using config: " << optarg << std::endl;
      slam_config_path = optarg;
      mslam::JsonConfig json_config(optarg);
      json_config.load();
      config = json_config.getConfig();
      break;
    }
    case 'h':
      printUsage(argv[0]);
      return 0;

    case '?':
      printUsage(argv[0]);
      return 1;
      break;
    }
  }

  std::cout << config << std::endl;

  // Create logger.
  const auto spdlog_logger = std::make_shared<spdlog::logger>(
      "mslam", std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  spdlog_logger->set_level(config.log_level);
  const auto logger = spdlog_logger;

  // Create Map interface.
  std::shared_ptr<mslam::IMap> map;
  auto voxel_map = std::make_shared<mslam::VoxelHashMap>(
      config.map_parameters.resolution,
      config.map_parameters.max_points_per_voxel);
  voxel_map->setNumAdjacentVoxelSearch(1); /// \todo add configurable?
  map = std::move(voxel_map);

  mslam::SlamServer slam_server(logger);
  slam_server.start();

  // Create sensor readers.
  std::shared_ptr<msensor::ILidar> lidar_sensor;
  std::shared_ptr<msensor::IImu> imu_sensor;

  if (config.remote_scanner == "local") {
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
    logger->info("Initialized local Mid360 using config: {}",
                 mid360_config_path.string());

  } else {
    auto remote = std::make_shared<SensorsRemoteClient>(config.remote_scanner);
    remote->init();
    remote->start();
    lidar_sensor = remote;
    imu_sensor = remote;
    logger->info("Connected to remote sensor server at {}",
                 config.remote_scanner);
  }

  mslam::Slam slam(logger, config, map);
  slam_server.setSlam(&slam);

  slam.run(lidar_sensor, imu_sensor, slam_server);

  return 0;
}
