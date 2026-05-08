#include "conversion.hh"

namespace mslam {

sensors::Pose3D toGRPC(const Pose3D &pose) {
  sensors::Pose3D msg;
  msg.set_x(pose[0]);
  msg.set_y(pose[1]);
  msg.set_z(pose[2]);
  msg.set_phi(pose[3]);
  msg.set_omega(pose[4]);
  msg.set_theta(pose[5]);
  return msg;
}

sensors::PointCloud3 toGRPC(const PointCloud3 &map) {
  sensors::PointCloud3 msg;
  msg.mutable_points()->Reserve(static_cast<int>(map.size()));
  for (const auto &point : map) {
    auto *grpc_point = msg.add_points();
    grpc_point->set_x(point.x);
    grpc_point->set_y(point.y);
    grpc_point->set_z(point.z);
  }
  return msg;
}

} // namespace mslam