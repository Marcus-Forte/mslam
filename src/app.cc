#include "ConsoleLogger.hh"
#include "FileScan.hh"
#include "ILog.hh"
#include "IScan.hh"
#include "RemoteMap.hh"
#include "RemoteScan.hh"
#include "TImer.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/KDtree2DMap.hh"
#include "points.pb.h"
#include <filesystem>
#include <fstream>
#include <getopt.h>
#include <memory>

int main(int argc, char **argv) {
  std::string filescan;
  int opt;
  bool use_filescan = false;
  while ((opt = getopt(argc, argv, "f:")) != -1) {
    switch (opt) {
    case 'f':
      filescan = optarg;
      use_filescan = true;
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
  while (true) {

    const auto scan = scanner->getScan(false);

    tmr.start();
    map.addScan(scan);
    auto delta_us = tmr.stop();
    logger.log(ILog::Level::INFO, "addScan: {} us", delta_us);
    // (optional) predict motion with high frequency IMU.
    // auto scan = rscan.getScan(true);
    // pre-process scan. Dedistort.

    // Register scan with map (optimization problem). Factor IMU measurements
    // in. Estimate correspondences.

    // Update map with transformed scan.
    const auto mapcloud = map.getPointCloudRepresentation();

    logger.log(ILog::Level::INFO, "Posting map... ({} pts)",
               mapcloud.points.size());

    rmap.publishMap(mapcloud);
    rmap.publishPose(pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}