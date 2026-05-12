#include "Conversion.hh"

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
  msg.set_timestamp(0);

  auto *x = msg.mutable_x();
  auto *y = msg.mutable_y();
  auto *z = msg.mutable_z();

  const auto point_count = static_cast<int>(map.size());
  x->Reserve(point_count);
  y->Reserve(point_count);
  z->Reserve(point_count);

  for (const auto &point : map) {
    x->Add(point.x);
    y->Add(point.y);
    z->Add(point.z);
  }
  return msg;
}

} // namespace mslam