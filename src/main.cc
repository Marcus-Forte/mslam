
#include "ConsoleLogger.hh"
#include "config/JsonConfig.hh"

#include "map/KDTreeMap.hh"
#include "map/VoxelHashMap.hh"
#include "sensors_remote_client.hh"
#include "slam/PointCloudExporter.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/Slam.hh"
#include "slam/SlamServer.hh"
#include <getopt.h>
#include <iostream>
#include <memory>
#include <stdexcept>

const unsigned int g_default_playback_delay_ms = 10;

namespace {

void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name
            << " [-c config.json] [-d delay_ms] [-f recording.pbscan] "
               "[-o output_prefix] [-h]\n"
            << "  -c <file>  Load SLAM configuration from JSON\n"
            << "  -d <ms>    Delay between playback entries when using -f\n"
            << "  -f <file>  Replay a recorded scan file instead of connecting "
               "remotely\n"
            << "  -o <path>  Save final point clouds as <path>_voxel_hash.ply "
               "and <path>_transformed_scans.ply\n"
            << "  -h         Show this help message\n";
}
} // namespace

int main(int argc, char **argv) {
  int opt;
  mslam::SlamConfiguration config;
  std::string slam_play_file = "";
  std::string ply_export_prefix;
  unsigned int playback_delay_ms = g_default_playback_delay_ms;
  while ((opt = getopt(argc, argv, "c:d:f:ho:")) != -1) {
    switch (opt) {
    case 'c': {
      std::cout << "Using config: " << optarg << std::endl;
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
    case 'o':
      ply_export_prefix = optarg;
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
  if (config.map_type == mslam::MapType::Voxel) {
    map = std::make_shared<mslam::VoxelHashMap>(
        config.map_parameters.resolution,
        config.map_parameters.max_points_per_voxel);
    reinterpret_cast<mslam::VoxelHashMap *>(map.get())
        ->setNumAdjacentVoxelSearch(1); /// \todo add configurable?
  } else {
    map = std::make_shared<mslam::KDTreeMap>(config.map_parameters.resolution);
  }

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
    /// \todo lidar factory
    throw std::runtime_error("Local mode not yet supported");

  } else {
    auto scanner = std::make_shared<SensorsRemoteClient>(config.remote_scanner);
    scanner->init();
    scanner->start();
    lidar_sensor = std::dynamic_pointer_cast<msensor::ILidar>(scanner);
    imu_sensor = std::dynamic_pointer_cast<msensor::IImu>(scanner);
  }

  mslam::Slam slam(logger, config, map);
  slam_server.setSlam(&slam);

  mslam::PointCloudExporter point_cloud_exporter(
      ply_export_prefix, config.map_parameters.resolution,
      config.map_parameters.max_points_per_voxel);

  slam.run(lidar_sensor, imu_sensor, slam_server, point_cloud_exporter,
           playback_player);

  if (point_cloud_exporter.isEnabled()) {
    point_cloud_exporter.save();
    logger->log(ILog::Level::INFO,
                "Saved final voxel-hash cloud to {} ({} points)",
                point_cloud_exporter.getVoxelHashPath().string(),
                point_cloud_exporter.getVoxelHashPointCount());
    logger->log(ILog::Level::INFO,
                "Saved accumulated transformed scans to {} ({} points)",
                point_cloud_exporter.getTransformedScansPath().string(),
                point_cloud_exporter.getTransformedScanPointCount());
  }

  return 0;
}