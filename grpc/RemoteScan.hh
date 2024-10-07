#pragma once

#include "IScan.hh"
#include "Points.hh"
#include "points.grpc.pb.h"
#include <future>
#include <grpcpp/channel.h>
#include <memory>
#include <queue>

class RemoteScan : public IScan {
public:
  RemoteScan(const std::string &remote_ip);
  void Start();
  void Stop();
  PointCloud2D getScan() override;

private:
  std::string remote_ip_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<lidar::LidarService::Stub> service_stub_;
  std::unique_ptr<grpc::ClientContext> service_context_;

  bool is_running_;
  std::future<void> read_thread_;

  std::queue<PointCloud2D> scans_;
};