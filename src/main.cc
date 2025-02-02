
#include "config/JsonConfig.hh"
#include "lidar/Mid360.hh"

#include "sensors_client.hh"
#include "slam/Slam.hh"
#include "timing/timing.hh"
#include <filesystem>
#include <getopt.h>
#include <memory>

int main(int argc, char **argv) {

  int opt;
  mslam::SlamConfiguration config;
  while ((opt = getopt(argc, argv, "c:")) != -1) {
    switch (opt) {
    case 'c':
      std::cout << "Using config: " << optarg << std::endl;
      mslam::JsonConfig json_config(optarg);
      json_config.load();
      config = json_config.getConfig();
      break;
    }
  }

  std::cout << config << std::endl;

  std::shared_ptr<msensor::ILidar> lidar_sensor;
  std::shared_ptr<msensor::IImu> imu_sensor;
  if (config.remote_scanner == "local") {
    /// \todo Select which lidar?
    throw std::runtime_error("Local mode not yet supported");

  } else {
    auto scanner = std::make_shared<SensorsClient>(config.remote_scanner);
    scanner->start();
    lidar_sensor = std::dynamic_pointer_cast<msensor::ILidar>(scanner);
    imu_sensor = std::dynamic_pointer_cast<msensor::IImu>(scanner);
  }

  mslam::Slam slam(config.parameters);

  while (true) {

    auto scan = lidar_sensor->getScan();
    auto imu = imu_sensor->getImuData();

    std::cout << "Scan: " << scan.points.size() << " @ " << scan.timestamp
              << std::endl;
    std::cout << "IMU ax " << imu.ax << " @ " << imu.timestamp << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  //   ConsoleLogger logger;

  //   mslam::Pose2D pose{0, 0, 0};

  //   std::unique_ptr<mslam::IMap2D> map;
  //   if (slam_config.map_type == mslam::MapType::Voxel) {
  //     map = std::make_unique<mslam::VoxelHashMap>(0.1, 2);
  //     reinterpret_cast<mslam::VoxelHashMap *>(map.get())
  //         ->setNumAdjacentVoxelSearch(2);
  //   } else {
  //     map = std::make_unique<mslam::KDTree2DMap>();
  //   }
  //   // Form a map with first 5 scans.

  //   for (int i = 0; i < 5; ++i) {
  //     // TODO type 2D,3D
  //     const auto scan = mslam::PointCloud2{}; // scanner->getScan();

  //     map->addScan(scan);
  //     logger.log(ILog::Level::INFO, "Init map pts: {}",
  //                map->getPointCloudRepresentation().size());
  //   }

  //   mslam::Registration registration;
  //   double last_imu_timestamp = 0;

  //   // constant velocity model
  //   Eigen::Vector2d vel = {0, 0};
  //   auto last_time = timing::getNowUs();
  //   while (true) {

  //     // Predict
  //     if (slam_config.with_imu) {
  //       const auto imu = scanner->getImuData();
  //       if (imu.timestamp != 0) {

  //         double imu_delta =
  //             static_cast<double>(imu.timestamp - last_imu_timestamp) /
  //             1000.0;
  //         if (imu_delta < 0.5) {

  //           pose[2] += imu.gz * imu_delta;
  //         }
  //         last_imu_timestamp = imu.timestamp;
  //       }
  //     }

  //     // Const velocity model
  //     auto current_time = timing::getNowUs();
  //     double deltatime = double(current_time - last_time) / 1000.0;
  //     pose[0] += vel[0] * deltatime;
  //     pose[1] += vel[1] * deltatime;
  //     last_time = current_time;
  //     // Update
  //     auto scan = mslam::PointCloud2{}; // TODO fix type
  //     scanner->getScan(); if (!scan.empty()) {
  //       // std::cout << scan.timestamp << std::endl;
  //       // Pre-process scan. Dedistort.

  //       // Register scan with map (optimization problem). Factor IMU
  //       // measurements.
  //       if (slam_config.with_lidar) {
  //         const auto new_pose = registration.Align(pose, *map, scan);
  //         const auto delta = new_pose - pose;
  //         vel[0] = delta[0];
  //         vel[1] = delta[1];
  //         pose = new_pose;
  //       }

  //       // Transform new points to updated frame.
  //       Eigen::Affine2d affine = toAffine(pose[0], pose[1], pose[2]);
  //       for (auto &p : scan) {
  //         p = affine * p;
  //       }

  //       // Update map with transformed scan.
  //       // map->addScan(scan.points);
  //       const auto mapcloud = map->getPointCloudRepresentation();
  //       logger.log(ILog::Level::INFO, "map pts: {}",
  //                  map->getPointCloudRepresentation().size());

  //       // rmap.publishScan(mapcloud, 1.0, 0.0, 0.0, "map");
  //       // rmap.publishScan(scan.points, 0, 1, 0, "scan");
  //       // rmap.publishPose(pose);

  //       std::cout << "Pose:" << pose << std::endl;
  //     }

  //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
  //   }
  // }
}