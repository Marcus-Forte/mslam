#pragma once

#include "IScan.hh"
#include "common/IMU.hh"
#include "common/Points.hh"
#include "sensors.grpc.pb.h"
#include <deque>
#include <future>
#include <grpcpp/channel.h>
#include <memory>

class RemoteScan : public mslam::IScan {
public:
  RemoteScan(const std::string &remote_ip);
  void Start();
  void Stop();
  mslam::PointCloud2T getScan(bool blocking = false) override;
  mslam::IMUData getImuData() override;

private:
  std::string remote_ip_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<sensors::SensorService::Stub> service_stub_;

  bool is_running_;
  std::future<void> read_thread_;
  std::future<void> imu_reader_thread_;

  std::deque<mslam::PointCloud2T> scans_;
  std::deque<mslam::IMUData> imu_measurements_;
};