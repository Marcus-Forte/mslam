#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/CorrespondenceFinder.hh"
#include "mslam/slam/PointCloudIO.hh"
#include "mslam/slam/Transform.hh"
#include "mslam/slam/registration/PointToPointRegistration.hh"
#include <filesystem>
#include <iostream>
#include <spdlog/spdlog.h>

void printUsage() {
  std::cout << "register_scans: <scan1.ply> <scan2.ply> ..." << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 3) {
    printUsage();
    exit(0);
  }

  const auto num_scans = argc - 1;
  auto logger = spdlog::default_logger();

  std::vector<mslam::PointCloud> scans;

  for (int i = 1; i < argc; ++i) {
    if (!std::filesystem::exists(argv[i])) {
      logger->error("File of source  does not exist: {}", argv[1]);
      exit(-1);
    }
    scans.emplace_back(mslam::readPlyPointCloud(argv[i]));
    logger->info("loaded points: source: {}", scans.back().size());
  }

  /// Add the first scan as a map
  auto map = std::make_shared<mslam::VoxelHashMap>(0.1F, 1);
  map->addScan(scans.front());

  mslam::PointToPointRegistration registration(
      50, 3, 0.5F, logger, std::make_shared<mslam::CorrespondenceFinder>());

  mslam::SlamState state;

  for (auto scan_idx = 1; scan_idx < num_scans; ++scan_idx) {
    state = registration.Align(state, *map, scans[scan_idx]);
    auto transformed_scan = scans[scan_idx];
    transformCloud(toAffine(state.position.x(), state.position.y(),
                            state.position.z(), state.rotation.x(),
                            state.rotation.y(), state.rotation.z()),
                   transformed_scan);
    map->addScan(transformed_scan);
    logger->info("Pose {}: x={}, y={}, z={}, theta={}", scan_idx,
                 state.position.x(), state.position.y(), state.position.z(),
                 state.rotation.z());
  }

  logger->info("Estimated transform: x: {}, y: {}, theta: {}",
               state.position.x(), state.position.y(), state.rotation.z());
}