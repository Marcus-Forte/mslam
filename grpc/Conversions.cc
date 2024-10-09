
#include "Conversions.hh"
#include "common/IMU.hh"
#include "glserver.pb.h"
#include "points.pb.h"

mslam::PointCloud2D fromGRPC(const lidar::PointCloud3 &msg) {
  mslam::PointCloud2D pointcloud;

  pointcloud.points.reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    pointcloud.points.emplace_back(pt.x(), pt.y());
  }
  pointcloud.timestamp = msg.timestamp();

  return pointcloud;
}

mslam::IMUData fromGRPC(const lidar::IMUData &msg) {

  return {msg.timestamp(), msg.ax(), msg.ay(), msg.az(),
          msg.gx(),        msg.gy(), msg.gz()};
}

gl::PointCloud3 toGRPC(const mslam::PointCloud2D &pointcloud) {
  gl::PointCloud3 grpc_cloud;
  for (const auto &pt : pointcloud.points) {
    auto *grpc_pt = grpc_cloud.add_points();
    grpc_pt->set_x(pt[0]);
    grpc_pt->set_y(pt[1]);
    grpc_pt->set_z(0);

    grpc_pt->set_r(1.0);
    grpc_pt->set_g(0.0);
    grpc_pt->set_b(0.0);
  }

  return grpc_cloud;
}
