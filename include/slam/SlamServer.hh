#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "slam.grpc.pb.h"
#include <condition_variable>
#include <grpcpp/grpcpp.h>
#include <memory>
#include <mutex>
#include <string>

namespace mslam {

class SlamServer : public sensors::SlamService::Service,
                   public sensors::ScanService::Service {
public:
  explicit SlamServer(std::shared_ptr<ILog> logger, std::string address = {});
  ~SlamServer();

  SlamServer(const SlamServer &) = delete;
  SlamServer &operator=(const SlamServer &) = delete;

  void start();
  void stop();

  void updatePose(const Pose3D &pose);
  void updateMap(const PointCloud3 &map);
  void updateScan(const PointCloud3 &scan);

private:
  grpc::Status
  GetMap(grpc::ServerContext *context, const sensors::Empty *,
         grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  grpc::Status
  GetScan(grpc::ServerContext *context, const sensors::Empty *,
          grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  grpc::Status GetPose(grpc::ServerContext *, const sensors::Empty *,
                       sensors::Pose3D *response) override;

  std::shared_ptr<ILog> logger_;
  std::string address_;
  std::unique_ptr<grpc::Server> server_;
  mutable std::mutex mutex_;
  mutable std::condition_variable map_cv_;
  mutable std::condition_variable scan_cv_;
  Pose3D pose_;
  PointCloud3 map_;
  PointCloud3 scan_;
  uint64_t map_version_ = 0;
  uint64_t scan_version_ = 0;
};

} // namespace mslam