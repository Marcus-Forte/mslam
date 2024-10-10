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

constexpr int g_idleTimeMs = 5;
constexpr int g_LidarQueueSize = 10;
constexpr int g_imuQueueSize = 100;

std::mutex g_mutex;
std::mutex g_imu_mutex;

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
    scans_.pop_front();
    return pointcloud;
  }

  return {};
}

mslam::IMUData RemoteScan::getImuData() {
  if (!is_running_) {
    throw std::runtime_error("Scan not started.");
  }

  if (!imu_measurements_.empty()) {
    std::lock_guard<std::mutex> lock(g_imu_mutex);
    const auto imu_data = imu_measurements_.front();
    imu_measurements_.pop_front();
    return imu_data;
  }

  return {};
}

void RemoteScan::Start() {

  is_running_ = true;

  read_thread_ = std::async(std::launch::async, [&]() {
    auto service_context_ = std::make_unique<grpc::ClientContext>();
    google::protobuf::Empty empty_response;
    auto reader =
        service_stub_->getScan(service_context_.get(), empty_response);

    lidar::PointCloud3 msg;
    while (is_running_) {
      if (!reader->Read(&msg)) {
        // std::cout << "Unable to read remote lidar" << std::endl;
      } else {
        std::lock_guard<std::mutex> lock(g_mutex);
        scans_.push_back(fromGRPC(msg));

        if (scans_.size() > g_LidarQueueSize) {
          scans_.pop_front();
        }
      }
    }
  });

  imu_reader_thread_ = std::async(std::launch::async, [&]() {
    auto imu_service_context_ = std::make_unique<grpc::ClientContext>();
    google::protobuf::Empty empty_response;
    auto imu_reader =
        service_stub_->getImu(imu_service_context_.get(), empty_response);
    lidar::IMUData msg;

    while (is_running_) {

      if (!imu_reader->Read(&msg)) {
        // std::cout << "Unable to read remote imu" << std::endl;
      } else {
        std::lock_guard<std::mutex> lock(g_imu_mutex);
        imu_measurements_.push_back(fromGRPC(msg));

        if (imu_measurements_.size() > g_imuQueueSize) {
          imu_measurements_.pop_front();
        }
      }
    }
  });
}

void RemoteScan::Stop() {
  is_running_ = false;
  read_thread_.get();
  imu_reader_thread_.get();
}
