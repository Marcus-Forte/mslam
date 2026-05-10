
#include "ConsoleLogger.hh"
#include "config/JsonConfig.hh"

#include "Timer.hh"
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

const int g_init_scans = 10;
const unsigned int g_default_playback_delay_ms = 10;

namespace {
mslam::PointCloud3 toPointCloud3(const mslam::VectorPoint3d &points) {
  mslam::PointCloud3 point_cloud;
  point_cloud.reserve(points.size());
  for (const auto &point : points) {
    point_cloud.emplace_back(point.x(), point.y(), point.z());
  }
  return point_cloud;
}

void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name
            << " [-c config.json] [-d delay_ms] [-f recording.pbscan] [-h]\n"
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
  std::string slam_play_file = "";
  unsigned int playback_delay_ms = g_default_playback_delay_ms;
  while ((opt = getopt(argc, argv, "c:d:f:h")) != -1) {
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
        ->setNumAdjacentVoxelSearch(1); /// \todo add configurable?
  } else {
    map = std::make_shared<mslam::KDTreeMap>();
  }

  mslam::SlamServer slam_server(logger);
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

  int init_scan_count = 0;
  Timer scan_timer;
  Timer stage_timer;

  while (true) {

    const auto scani = lidar_sensor->getScan();
    if (!scani) {
      if (playback_player && playback_player->isFinished()) {
        logger->log(ILog::Level::INFO,
                    "Playback exhausted; exiting SLAM process.");
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    scan_timer.start();

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

    if (init_scan_count < g_init_scans) {
      stage_timer.start();
      auto filtered_scan = preprocessor->removePointsNearCenter(scan);
      map->addScan(*filtered_scan->points);
      const auto map_update_us = stage_timer.stop();
      init_scan_count++;
      slam_server.updateScan(*filtered_scan->points);
      slam_server.updateTransformedScan(*filtered_scan->points);
      slam_server.updateMap(map->getPointCloudRepresentation());
      logger->log(ILog::Level::INFO, "Init scan {}/{}. Map points: {}",
                  init_scan_count, g_init_scans,
                  map->getPointCloudRepresentation().size());
      logger->log(ILog::Level::INFO,
                  "Init timing. Map update: {} us. Total: {} us", map_update_us,
                  scan_timer.stop());
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

        stage_timer.start();
        slam.Predict(*imudata);
        const auto imu_predict_us = stage_timer.stop();
        slam_server.updatePose(slam.getPose());
        logger->log(ILog::Level::INFO, "IMU predict took: {} us",
                    imu_predict_us);
      }
    }
    if (config.with_lidar) {
      static uint64_t last_scan_time = 0;
      // print delta
      if (last_scan_time != 0) {
        logger->log(ILog::Level::INFO, "Scan delta: {} ms",
                    (scan.timestamp - last_scan_time) * 1e-6);
      }
      last_scan_time = scan.timestamp;

      logger->log(ILog::Level::INFO, "Processing scan with {} points @ {}",
                  scan.points->size(), scan.timestamp);

      stage_timer.start();
      auto filtered_scan = preprocessor->removePointsNearCenter(scan);
      const auto range_filter_us = stage_timer.stop();

      logger->log(ILog::Level::INFO,
                  "Removed near-center points: {} -> {} points",
                  scan.points->size(), filtered_scan->points->size());

      stage_timer.start();
      filtered_scan = preprocessor->downsample(*filtered_scan);
      const auto downsample_us = stage_timer.stop();
      const auto preprocessor_us = range_filter_us + downsample_us;

      logger->log(ILog::Level::INFO,
                  "Preprocess benchmark. Range filter: {} us. Downsample: {} "
                  "us",
                  range_filter_us, downsample_us);
      logger->log(ILog::Level::INFO, "Downsampled scan: {} -> {} points",
                  filtered_scan->points->size(), filtered_scan->points->size());

      stage_timer.start();
      slam.Update(*filtered_scan);
      const auto registration_us = stage_timer.stop();

      const auto &correspondences = slam.getLastMapCorrespondences();

      stage_timer.start();
      slam_server.updateCorrespondences(toPointCloud3(correspondences));
      slam_server.updateScan(*filtered_scan->points);
      slam_server.updatePose(slam.getPose());

      transformCloud(slam.getTransform(), *filtered_scan->points);
      const auto transform_us = stage_timer.stop();

      map->addScan(*filtered_scan->points);

      stage_timer.start();
      slam_server.updateTransformedScan(*filtered_scan->points);
      const auto transformed_scan_publish_us = stage_timer.stop();

      stage_timer.start();

      logger->log(ILog::Level::INFO,
                  "Scan timing. Preprocess: {} us. Registration: {} us. "
                  "Transform: {} us. Publish transformed scan: {} us. "
                  "Total: {} us",
                  preprocessor_us, registration_us, transform_us,
                  transformed_scan_publish_us, scan_timer.stop());
    }
  }

  return 0;
}