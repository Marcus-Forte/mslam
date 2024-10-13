
#include "Conversions.hh"
#include "common/IMU.hh"
#include "gl_server.pb.h"
#include "sensors.pb.h"

mslam::PointCloud2T fromGRPC(const sensors::PointCloud3 &msg) {
  mslam::PointCloud2T pointcloud;

  pointcloud.points.reserve(msg.points_size());
  for (const auto &pt : msg.points()) {
    pointcloud.points.emplace_back(pt.x(), pt.y());
  }
  pointcloud.timestamp = msg.timestamp();

  return pointcloud;
}

mslam::IMUData fromGRPC(const sensors::IMUData &msg) {

  return {msg.timestamp(), msg.ax(), msg.ay(), msg.az(),
          msg.gx(),        msg.gy(), msg.gz()};
}

gl::PointCloud3 toGRPC(const mslam::PointCloud2 &pointcloud, float r, float g,
                       float b) {
  gl::PointCloud3 grpc_cloud;
  for (const auto &pt : pointcloud) {
    auto *grpc_pt = grpc_cloud.add_points();
    grpc_pt->set_x(pt[0]);
    grpc_pt->set_y(pt[1]);
    grpc_pt->set_z(0);

    grpc_pt->set_r(r);
    grpc_pt->set_g(g);
    grpc_pt->set_b(b);
  }

  return grpc_cloud;
}
