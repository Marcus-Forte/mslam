#pragma once

#include "IMapPublisher.hh"
#include "glserver.grpc.pb.h"
#include <grpcpp/channel.h>

class RemoteMap : public mslam::IMapPublisher {
public:
  RemoteMap(const std::string &remote_ip);
  void publishScan(const mslam::PointCloud2D &map, float r, float g, float b,
                   const std::string &name) const override;
  void publishPose(const mslam::Pose2D &pose) const override;

private:
  std::string remote_ip_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<gl::addToScene::Stub> service_stub_;
  std::unique_ptr<grpc::ClientContext> service_context_;
};