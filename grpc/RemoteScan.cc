#include "RemoteScan.hh"
#include "Conversions.hh"
#include "common/Points.hh"
#include "points.grpc.pb.h"
#include "points.pb.h"
#include <chrono>
#include <future>
#include <google/protobuf/empty.pb.h>
#include <grpcpp/client_context.h>
#include <grpcpp/grpcpp.h>
#include <mutex>
#include <thread>

const int g_idleTimeMs = 5;
std::mutex g_mutex;

RemoteScan::RemoteScan(const std::string &remote_ip)
    : remote_ip_(remote_ip), is_running_(false) {

  channel_ = grpc::CreateChannel(remote_ip, grpc::InsecureChannelCredentials());
  service_stub_ = lidar::LidarService::NewStub(channel_);
}

mslam::PointCloud2D RemoteScan::getScan(bool blocking) {
  if (!is_running_) {
    throw std::runtime_error("Scan not started.");
  }

  if (blocking) {
    while (scans_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(g_idleTimeMs));
    }
  }

  if (!scans_.empty()) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto pointcloud = scans_.front();
    pointcloud.timestamp =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    scans_.pop();
    return pointcloud;
  }

  return {};
}

void RemoteScan::Start() {

  is_running_ = true;

  // Reader thread
  read_thread_ = std::async(std::launch::async, [&]() {
    service_context_ = std::make_unique<grpc::ClientContext>();
    google::protobuf::Empty empty_response;
    auto reader =
        service_stub_->getScan(service_context_.get(), empty_response);
    lidar::PointCloud3 msg;
    while (is_running_) {

      if (!reader->Read(&msg)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Unable to read remote lidar" << std::endl;
        auto state = channel_->GetState(true);

        std::cout << "State: " << state << std::endl;
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(g_mutex);
        scans_.push(fromGRPC(msg));
      }
    }
  });
}

void RemoteScan::Stop() {
  is_running_ = false;
  read_thread_.get();
}
