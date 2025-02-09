
#include "ConsoleLogger.hh"
#include "config/JsonConfig.hh"

#include "map/KDTreeMap.hh"
#include "map/VoxelHashMap.hh"
#include "sensors_client.hh"
#include "slam/Slam.hh"
#include "slam/SlamPlayer.hh"
#include <getopt.h>
#include <memory>

const int g_init_scans = 5;

int main(int argc, char **argv) {

  int opt;
  mslam::SlamConfiguration config;
  std::string slam_play_file = "";
  while ((opt = getopt(argc, argv, "c:f:")) != -1) {
    switch (opt) {
    case 'c': {
      std::cout << "Using config: " << optarg << std::endl;
      mslam::JsonConfig json_config(optarg);
      json_config.load();
      config = json_config.getConfig();
      break;
    }
    case 'f':
      std::cout << "Using SlamPlayer with: " << optarg << std::endl;
      slam_play_file = optarg;
      break;

    case '?':
      exit(0);
      break;
    }
  }

  std::cout << config << std::endl;

  // Create logger.
  const auto logger = std::make_shared<ConsoleLogger>();
  logger->setLevel(ILog::Level::INFO);

  // Create Map interface.
  std::shared_ptr<mslam::IMap> map;
  if (config.map_type == mslam::MapType::Voxel) {
    map = std::make_shared<mslam::VoxelHashMap>(0.1, 5);
    reinterpret_cast<mslam::VoxelHashMap *>(map.get())
        ->setNumAdjacentVoxelSearch(2); /// \todo add configurable
  } else {
    map = std::make_shared<mslam::KDTreeMap>();
  }
  if (!slam_play_file.empty()) {
    mslam::SlamPlayer player(slam_play_file, logger, config, map);
    player.run();
    exit(0);
  }

  // Create sensor readers.
  std::shared_ptr<msensor::ILidar> lidar_sensor;
  std::shared_ptr<msensor::IImu> imu_sensor;

  if (config.remote_scanner == "local") {
    /// \todo lidar factory
    throw std::runtime_error("Local mode not yet supported");

  } else {
    auto scanner = std::make_shared<SensorsClient>(config.remote_scanner);
    scanner->start();
    lidar_sensor = std::dynamic_pointer_cast<msensor::ILidar>(scanner);
    imu_sensor = std::dynamic_pointer_cast<msensor::IImu>(scanner);
  }

  mslam::Slam slam(logger, config.parameters, map);

  const int init_scans = 5;
  int init_scan_count = 0;

  while (true) {

    const auto scani = lidar_sensor->getScan();

    msensor::Scan3D scan;
    scan.points.reserve(scani.points.size());
    scan.timestamp = scani.timestamp;
    for (const auto &pt : scani.points) {
      scan.points.emplace_back(pt.x, pt.y, pt.z);
    }

    if (scan.points.size() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (init_scan_count < init_scans) {
      map->addScan(scan.points);
      init_scan_count++;
      logger->log(ILog::Level::INFO, "Init scan {}/{}. Map points: {}",
                  init_scan_count, init_scans,
                  map->getPointCloudRepresentation().size());
      continue;
    }

    if (config.with_imu) {
      msensor::IMUData imudata;
      while ((imudata = imu_sensor->getImuData()).timestamp != 0) {
        slam.Predict(imudata);
      }
    }
    if (config.with_lidar) {
      slam.Update(scan);
    }
  }
}