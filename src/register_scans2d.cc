#include "ConsoleLogger.hh"
#include "common/Points.hh"
#include "common/State.hh"
#include "map/VoxelHashMap.hh"
#include "slam/CorrespondenceFinder.hh"
#include "slam/PointCloudIO.hh"
#include "slam/Transform.hh"
#include "slam/registration/PointToPointRegistration.hh"
#include <filesystem>
#include <iostream>

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

  std::vector<mslam::PointCloud> scans;

  for (int i = 1; i < argc; ++i) {
    if (!std::filesystem::exists(argv[i])) {
      logger->log(ILog::Level::ERROR, "File of source  does not exist: {}",
                  argv[1]);
      exit(-1);
    }
    scans.emplace_back(mslam::readPlyPointCloud(argv[i]));
    logger->log(ILog::Level::INFO, "loaded points: source: {}",
                scans.back().size());
  }

  /// Add the first scan as a map
  auto map = std::make_shared<mslam::VoxelHashMap>(0.1F, 1);
  map->addScan(scans.front());

  mslam::PointToPointRegistration registration(
      50, 3, 0.5F, logger,
      std::make_shared<mslam::CorrespondenceFinder>(logger));

  mslam::SlamState state;

  for (auto scan_idx = 1; scan_idx < num_scans; ++scan_idx) {
    state = registration.Align(state, *map, scans[scan_idx]);
    auto transformed_scan = scans[scan_idx];
    transformCloud(toAffine(state.position.x(), state.position.y(),
                            state.position.z(), state.rotation.x(),
                            state.rotation.y(), state.rotation.z()),
                   transformed_scan);
    map->addScan(transformed_scan);
    logger->log(ILog::Level::INFO, "Pose {}: x={}, y={}, z={}, theta={}",
                scan_idx, state.position.x(), state.position.y(),
                state.position.z(), state.rotation.z());
  }

  logger->log(ILog::Level::INFO, "Estimated transform: x: {}, y: {}, theta: {}",
              state.position.x(), state.position.y(), state.rotation.z());
}