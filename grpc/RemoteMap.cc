#include "RemoteMap.hh"
#include "Conversions.hh"
#include "gl_server.grpc.pb.h"
#include <google/protobuf/empty.pb.h>
#include <grpcpp/client_context.h>
#include <grpcpp/grpcpp.h>

RemoteMap::RemoteMap(const std::string &remote_ip) : remote_ip_(remote_ip) {
  channel_ = grpc::CreateChannel(remote_ip, grpc::InsecureChannelCredentials());
  service_stub_ = gl::addToScene::NewStub(channel_);
}
void RemoteMap::publishScan(const mslam::PointCloud2 &map, float r, float g,
                            float b, const std::string &name) const {
  grpc::ClientContext context;
  google::protobuf::Empty empty;

  auto cloud = toGRPC(map, r, g, b);
  cloud.set_entity_name(name);

  service_stub_->addPointCloud(&context, cloud, &empty);
}

void RemoteMap::publishPose(const mslam::Pose2D &pose) const {
  static grpc::ClientContext context;
  google::protobuf::Empty empty;
  static auto writter = service_stub_->streamNamedPoints(&context, &empty);

  if (!writter) {
    std::cerr << "named point Writter error" << std::endl;
  }
  gl::NamedPoint3 msg;
  msg.set_name("pose");
  auto *pt = msg.mutable_point();
  pt->set_x(pose[0]);
  pt->set_y(pose[1]);
  pt->set_z(0);
  pt->set_g(1.0);
  msg.set_size(20);
  writter->Write(msg);
}