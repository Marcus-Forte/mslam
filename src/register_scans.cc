#include "ConsoleLogger.hh"
#include "map/KDTreeMap.hh"
#include "slam/Registration.hh"
#include <filesystem>
#include <pcl/io/ply_io.h>

void printUsage() {
  std::cout << "register_scans: <scan1.ply> <scan2.ply> ..." << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 3) {
    printUsage();
    exit(0);
  }

  const auto num_scans = argc - 1;
  auto logger = std::make_shared<ConsoleLogger>();

  std::vector<msensor::PointCloud3> scans;

  for (int i = 1; i < argc; ++i) {
    if (!std::filesystem::exists(argv[i])) {
      logger->log(ILog::Level::ERROR, "File of source  does not exist: {}",
                  argv[1]);
      exit(-1);
    }
    msensor::PointCloud3 &&scan{};
    pcl::io::loadPLYFile(argv[i], scan);
    scans.emplace_back(scan);
    logger->log(ILog::Level::INFO, "loaded points: source: {}",
                scans.back().size());
  }

  /// Add the first scan as a map
  auto map = std::make_shared<mslam::KDTreeMap>();
  map->addScan(scans[0]);

  mslam::Registration registration(50, 3, 0.5, logger);

  mslam::Pose2D pose{0, 0, 0};

  for (auto scan_idx = 1; scan_idx < num_scans; ++scan_idx) {

    pose = registration.Align2D(pose, *map, scans[scan_idx]);
    logger->log(ILog::Level::INFO, "Pose {}: x={}, y={}, theta={}", scan_idx,
                pose[0], pose[1], pose[2]);
  }

  logger->log(ILog::Level::INFO, "Estimated transform: x: {}, y: {}, theta: {}",
              pose[0], pose[1], pose[2]);
}