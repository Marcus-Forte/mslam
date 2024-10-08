#include "ConsoleLogger.hh"
#include "ILog.hh"
#include "RemoteMap.hh"
#include "RemoteScan.hh"
#include "common/Points.hh"
#include "map/Simple2DMap.hh"

#include "common/Pose.hh"
#include <chrono>
#include <cstdlib>
#include <thread>

int main(int argc, char **argv) {
  ConsoleLogger logger;

  RemoteScan rscan("192.168.2.34:50051");
  RemoteMap rmap("host.docker.internal:50052");

  rscan.Start();

  mslam::Pose2D pose{0, 0, 0};
  mslam::SimpleMap2D map;
  // Form a map with first 5 scans.

  for (int i = 0; i < 5; ++i) {
    const auto scan = rscan.getScan(true);
    map.addScan(scan);
    logger.log(ILog::Level::INFO, "Init map pts: {}",
               map.getPointCloudRepresentation().points.size());
  }
  while (true) {

    const auto scan = rscan.getScan(true);

    map.addScan(scan);
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