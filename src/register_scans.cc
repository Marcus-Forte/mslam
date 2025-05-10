#include "ConsoleLogger.hh"
#include "gl_publisher.hh"
#include "map/KDTreeMap.hh"
#include "map/VoxelHashMap.hh"
#include "slam/Registration.hh"
#include "slam/Transform.hh"
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <thread>

void registrationCallbackFun(const mslam::Pose2D &pose,
                             const mslam::VectorPoint2d &map_corrs,
                             const mslam::VectorPoint2d &src_corrs,
                             msensor::GLPublisher &gl_publisher) {
  {
    // Draw lines
    msensor::PointCloud3 src_lines;
    msensor::PointCloud3 tgt_lines;

    msensor::PointCloud3 src_cloud;
    const auto &&transform = toAffine(pose[0], pose[1], pose[2]);

    for (const auto &point : src_corrs) {
      const auto &&transformed_point =
          transform * Eigen::Vector2d(point[0], point[1]);

      src_cloud.emplace_back(transformed_point.x(), transformed_point.y(), 0);
    }

    gl_publisher.publishScan(src_cloud, 0.0, 1.0, 0.0, "source");
    gl_publisher.publishPose(pose[0], pose[1], 0.0);

    for (int i = 0; i < src_corrs.size(); ++i) {

      const auto &&transformed_source =
          transform * Eigen::Vector2d(src_corrs[i].x(), src_corrs[i].y());

      src_lines.emplace_back(transformed_source.x(), transformed_source.y(), 0);
      tgt_lines.emplace_back(map_corrs[i].x(), map_corrs[i].y(), 0);
    }

    gl_publisher.publishLines(src_lines, tgt_lines, 1.0, 1.0, 0.0);

    std::cin.get();
  }
}
void printUsage() {
  std::cout << "register_scans: <gl server address:port> <scan1.ply> "
               "<scan2.ply> ..."
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 4) {
    printUsage();
    exit(0);
  }

  const auto server_address = argv[1];
  const auto num_scans = argc - 2;
  auto logger = std::make_shared<ConsoleLogger>();

  std::vector<msensor::PointCloud3> scans;

  for (int i = 2; i < argc; ++i) {
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

  msensor::GLPublisher gl_publisher(server_address);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  gl_publisher.publishPose(0.f, 0.0, 0.0, 0.0, 0.2, 1.0, "original_pose");
  gl_publisher.publishScan(map->getPointCloudRepresentation(), 1.0, 0.0, 0.0,
                           "map");

  registration.registerIterationCallback2D(
      [&gl_publisher](const mslam::Pose2D &pose,
                      const mslam::VectorPoint2d &map_corrs,
                      const mslam::VectorPoint2d &src_corrs) {
        registrationCallbackFun(pose, map_corrs, src_corrs, gl_publisher);
      });

  mslam::Pose2D pose{0, 0, 0};

  for (auto scan_idx = 1; scan_idx < num_scans; ++scan_idx) {

    pose = registration.Align2D(pose, *map, scans[scan_idx]);
    // msensor::PointCloud3 transformed_scan = scans[scan_idx];
    // const auto transform = toAffine(pose[0], pose[1], pose[2]);
    // transformCloud(transform, transformed_scan);
    // map->addScan(transformed_scan);
    gl_publisher.publishScan(map->getPointCloudRepresentation(), 1.0, 0.0, 0.0,
                             "map");

    gl_publisher.publishPose(pose[0], pose[1], 0.0, 0.0, 0.2, 1.0,
                             "pose_" + std::to_string(scan_idx));
  }

  gl_publisher.publishScan(map->getPointCloudRepresentation(), 1.0, 0.0, 0.0,
                           "map");

  logger->log(ILog::Level::INFO, "Estimated transform: x: {}, y: {}, theta: {}",
              pose[0], pose[1], pose[2]);
}