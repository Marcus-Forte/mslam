#include "ConsoleLogger.hh"
#include "FileScan.hh"
#include "ILog.hh"
#include "IScan.hh"
#include "Registration.hh"
#include "RemoteMap.hh"
#include "RemoteScan.hh"
#include "Timer.hh"
#include "Transform.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/KDtree2DMap.hh"
#include <chrono>
#include <filesystem>
#include <getopt.h>
#include <memory>

uint64_t now() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char **argv) {
  std::string filescan;
  int opt;
  bool use_filescan = false;
  bool use_imu = false;
  bool use_lidar = false;
  while ((opt = getopt(argc, argv, "lif:")) != -1) {
    switch (opt) {
    case 'f':
      filescan = optarg;
      use_filescan = true;
      break;
    case 'i':
      std::cout << "Using IMU..." << std::endl;
      use_imu = true;
      break;
    case 'l':
      use_lidar = true;
      std::cout << "Using LiDAR..." << std::endl;
      break;
    }
  }
  std::unique_ptr<mslam::IScan> scanner;

  if (use_filescan) {

    if (!std::filesystem::exists(filescan)) {
      std::cerr << "File: " << filescan << " does not exist." << std::endl;
      exit(-1);
    }

    scanner = std::make_unique<mslam::FileScan>(filescan);
  } else {
    scanner = std::make_unique<RemoteScan>("192.168.2.34:50051");
    static_cast<RemoteScan *>(scanner.get())->Start();
  }

  ConsoleLogger logger;

  RemoteMap rmap("host.docker.internal:50052");

  mslam::Pose2D pose{0, 0, 0};
  mslam::KDTree2DMap map;
  // Form a map with first 5 scans.

  for (int i = 0; i < 5; ++i) {
    const auto scan = scanner->getScan(true);
    map.addScan(scan);
    logger.log(ILog::Level::INFO, "Init map pts: {}",
               map.getPointCloudRepresentation().points.size());
  }

  Timer tmr;
  mslam::Registration registration;
  double last_imu_timestamp = 0;

  // constant velocity model
  Eigen::Vector2d vel = {0, 0};
  auto last_time = now();
  while (true) {

    // Predict
    if (use_imu) {
      const auto imu = scanner->getImuData();
      if (imu.timestamp != 0) {

        double imu_delta =
            static_cast<double>(imu.timestamp - last_imu_timestamp) / 1000.0;
        if (imu_delta < 0.5) {

          pose[2] += imu.gz * imu_delta;
        }
        last_imu_timestamp = imu.timestamp;
      }
    }

    // Const velocity model
    auto current_time = now();
    double deltatime = double(current_time - last_time) / 1000.0;
    pose[0] += vel[0] * deltatime;
    pose[1] += vel[1] * deltatime;
    last_time = current_time;

    // Update
    auto scan = scanner->getScan(false);
    if (!scan.points.empty()) {
      std::cout << scan.timestamp << std::endl;
      // Pre-process scan. Dedistort.

      // Register scan with map (optimization problem). Factor IMU measurements.
      if (use_lidar) {
        const auto new_pose = registration.Align(pose, map, scan);
        const auto delta = new_pose - pose;
        vel[0] = delta[0];
        vel[1] = delta[1];
        pose = new_pose;
      }

      // Transform new points to updated frame.
      Eigen::Affine2d affine = toAffine(pose[0], pose[1], pose[2]);
      for (auto &p : scan.points) {
        p = affine * p;
      }

      // Update map with transformed scan.
      // map.addScan(scan);
      const auto mapcloud = map.getPointCloudRepresentation();
      logger.log(ILog::Level::INFO, "map pts: {}",
                 map.getPointCloudRepresentation().points.size());

      rmap.publishScan(mapcloud, 1.0, 0.0, 0.0, "map");
      rmap.publishScan(scan, 0, 1, 0, "scan");
      rmap.publishPose(pose);

      std::cout << "Pose:" << pose << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}