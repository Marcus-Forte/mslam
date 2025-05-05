#include "ConsoleLogger.hh"
#include "gl_publisher.hh"
#include "map/KDTreeMap.hh"
#include "slam/Registration.hh"
#include "slam/Transform.hh"
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <thread>

void printUsage() {
  std::cout << "registration: <source> <target> <gl server address:port>"
            << std::endl;
}

int main(int argc, char **argv) {

  if (argc < 4) {
    printUsage();
    exit(0);
  }
  auto logger = std::make_shared<ConsoleLogger>();

  if (!std::filesystem::exists(argv[1])) {
    logger->log(ILog::Level::ERROR, "File of source cloud does not exist: {}",
                argv[1]);
    exit(-1);
  }

  if (!std::filesystem::exists(argv[2])) {
    logger->log(ILog::Level::ERROR, "File of target cloud does not exist: {}",
                argv[1]);
    exit(-1);
  }

  // Load points
  msensor::PointCloud3 source, target;
  pcl::io::loadPLYFile(argv[1], source);
  pcl::io::loadPLYFile(argv[2], target);

  logger->log(ILog::Level::INFO, "loaded points: source: {} target: {}",
              source.size(), target.size());

  /// Add the Target scan to Map
  auto map = std::make_shared<mslam::KDTreeMap>();
  map->addScan(target);

  mslam::Registration registration(50, 5, 0.5, logger);

  msensor::GLPublisher gl_publisher(argv[3]);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  gl_publisher.publishPose(0.f, 0.0, 0.0, 0.0, 0.2, 1.0, "original_pose");
  gl_publisher.publishScan(target, 1.0, 0.0, 0.0, "target");

  registration.registerIterationCallback2D(
      [&](const mslam::Pose2D &pose, const mslam::VectorPoint2d &map_corrs,
          const mslam::VectorPoint2d &src_corrs) {
        // Draw lines
        msensor::PointCloud3 src_lines;
        msensor::PointCloud3 tgt_lines;

        msensor::PointCloud3 src_cloud;
        const auto &&transform = toAffine(pose[0], pose[1], pose[2]);

        for (const auto &point : source) {
          const auto &&transformed_point =
              transform * Eigen::Vector2d(point.x, point.y);

          src_cloud.emplace_back(transformed_point.x(), transformed_point.y(),
                                 0);
        }

        gl_publisher.publishScan(src_cloud, 0.0, 1.0, 0.0, "source");
        gl_publisher.publishPose(pose[0], pose[1], 0.0);

        for (int i = 0; i < src_corrs.size(); ++i) {

          const auto &&transformed_source =
              transform * Eigen::Vector2d(src_corrs[i].x(), src_corrs[i].y());

          src_lines.emplace_back(transformed_source.x(), transformed_source.y(),
                                 0);
          tgt_lines.emplace_back(map_corrs[i].x(), map_corrs[i].y(), 0);
        }

        gl_publisher.publishLines(src_lines, tgt_lines, 1.0, 1.0, 0.0);

        std::cin.get();
      });
  mslam::Pose2D pose{0, 0, 0};

  pose = registration.Align2D(pose, *map, source);

  logger->log(ILog::Level::INFO, "Estimated transform: x: {}, y: {}, theta: {}",
              pose[0], pose[1], pose[2]);
}