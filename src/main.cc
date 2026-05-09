
#include "ConsoleLogger.hh"
#include "config/JsonConfig.hh"

#include "map/KDTreeMap.hh"
#include "map/VoxelHashMap.hh"
#include "sensors_remote_client.hh"
#include "slam/Preprocessor.hh"
#include "slam/RecordingSensorPlayer.hh"
#include "slam/Slam.hh"
#include "slam/SlamServer.hh"
#include "slam/Transform.hh"
#include <getopt.h>
#include <iostream>
#include <memory>
#include <thread>

const int g_init_scans = 5;

namespace {
void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name
            << " [-c config.json] [-f recording.pbscan] [-h]\n"
            << "  -c <file>  Load SLAM configuration from JSON\n"
            << "  -f <file>  Replay a recorded scan file instead of connecting "
               "remotely\n"
            << "  -h         Show this help message\n";
}
} // namespace

int main(int argc, char **argv) {
  int opt;
  mslam::SlamConfiguration config;
  std::string slam_play_file = "";
  while ((opt = getopt(argc, argv, "c:f:h")) != -1) {
    switch (opt) {
    case 'c': {
      std::cout << "Using config: " << optarg << std::endl;
      mslam::JsonConfig json_config(optarg);
      json_config.load();
      config = json_config.getConfig();
      break;
    }
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
        ->setNumAdjacentVoxelSearch(2); /// \todo add configurable
  } else {
    map = std::make_shared<mslam::KDTreeMap>();
  }

  mslam::SlamServer slam_server(logger);
  slam_server.start();

  // Create sensor readers.
  std::shared_ptr<msensor::ILidar> lidar_sensor;
  std::shared_ptr<msensor::IImu> imu_sensor;

  if (!slam_play_file.empty()) {
    auto player = std::make_shared<mslam::RecordingSensorPlayer>(
        slam_play_file, logger, config);
    player->init();
    player->startSampling();
    lidar_sensor = std::dynamic_pointer_cast<msensor::ILidar>(player);
    imu_sensor = std::dynamic_pointer_cast<msensor::IImu>(player);
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

  auto preprocessor =
      std::make_shared<mslam::Preprocessor>(config.preprocessor);
  mslam::Slam slam(logger, config.parameters, map);
  slam_server.updatePose(slam.getPose());

  const int init_scans = 5;
  int init_scan_count = 0;

  while (true) {

    const auto scani = lidar_sensor->getScan();
    if (!scani) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    msensor::Scan3D scan;
    scan.points->reserve(scani->points->size());
    scan.timestamp = scani->timestamp;
    for (const auto &pt : *scani->points) {
      scan.points->emplace_back(pt.x, pt.y, pt.z);
    }

    if (scan.points->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (init_scan_count < init_scans) {
      map->addScan(*scan.points);
      init_scan_count++;
      slam_server.updateScan(*scan.points);
      slam_server.updateMap(map->getPointCloudRepresentation());
      logger->log(ILog::Level::INFO, "Init scan {}/{}. Map points: {}",
                  init_scan_count, init_scans,
                  map->getPointCloudRepresentation().size());
      continue;
    }

    if (config.with_imu) {
      static uint64_t last_imu_time = 0;

      while (true) {
        const auto imudata = imu_sensor->getImuData();
        if (!imudata.has_value()) {
          break;
        }
        if (last_imu_time != 0) {
          logger->log(ILog::Level::INFO, "IMU delta: {} ms",
                      (imudata->timestamp - last_imu_time) * 1e-6);
        }
        last_imu_time = imudata->timestamp;

        logger->log(ILog::Level::INFO, "Processing IMU with @ {}",
                    imudata->timestamp);

        slam.Predict(*imudata);
        slam_server.updatePose(slam.getPose());
      }
    }
    if (config.with_lidar) {
      static uint64_t last_scan_time = 0;
      // print delta
      if (last_scan_time != 0) {
        logger->log(ILog::Level::DEBUG, "Scan delta: {} ms",
                    (scan.timestamp - last_scan_time) * 1e-6);
      }
      last_scan_time = scan.timestamp;

      logger->log(ILog::Level::INFO, "Processing scan with {} points @ {}",
                  scan.points->size(), scan.timestamp);
      auto filtered_scan = preprocessor->downsample(scan);
      logger->log(ILog::Level::INFO, "Downsampled scan to {} points",
                  filtered_scan->points->size());
      slam.Update(*filtered_scan);
      transformCloud(slam.getTransform(), *filtered_scan->points);
      slam_server.updateScan(*filtered_scan->points);
      map->addScan(*filtered_scan->points);
      slam_server.updateMap(map->getPointCloudRepresentation());
      slam_server.updatePose(slam.getPose());
    }
  }
}