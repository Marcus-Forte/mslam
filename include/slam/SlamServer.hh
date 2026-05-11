#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/IMap.hh"
#include "slam.grpc.pb.h"
#include <atomic>
#include <condition_variable>
#include <grpcpp/grpcpp.h>
#include <memory>
#include <mutex>
#include <string>

namespace mslam {

class SlamServer : public sensors::SlamService::Service {
public:
  explicit SlamServer(std::shared_ptr<ILog> logger, std::shared_ptr<IMap> map,
                      std::string address = {});
  ~SlamServer();

  SlamServer(const SlamServer &) = delete;
  SlamServer &operator=(const SlamServer &) = delete;

  void start();
  void stop();

  void updatePose(const Pose3D &pose);
  void updateMapIncrement(const PointCloud3 &increment);
  void updateTransformedScan(const PointCloud3 &scan);
  void updateCorrespondences(const PointCloud3 &correspondences);

private:
  grpc::Status GetMap(grpc::ServerContext *, const sensors::Empty *,
                      sensors::PointCloud3 *response) override;

  grpc::Status
  GetMapIncrements(grpc::ServerContext *context, const sensors::Empty *,
                   grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  grpc::Status
  GetTransformedScan(grpc::ServerContext *context, const sensors::Empty *,
                     grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  grpc::Status
  GetCorrespondences(grpc::ServerContext *context, const sensors::Empty *,
                     grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  grpc::Status GetPose(grpc::ServerContext *context, const sensors::Empty *,
                       grpc::ServerWriter<sensors::Pose3D> *writer) override;

  std::shared_ptr<ILog> logger_;
  std::shared_ptr<IMap> map_;
  std::string address_;
  std::unique_ptr<grpc::Server> server_;
  std::atomic<bool> stopping_{false};
  mutable std::mutex mutex_;

  mutable std::condition_variable map_increment_cv_;
  mutable std::condition_variable transformed_scan_cv_;
  mutable std::condition_variable correspondences_cv_;
  mutable std::condition_variable pose_cv_;
  Pose3D pose_;
  PointCloud3 map_increment_;
  PointCloud3 transformed_scan_;
  PointCloud3 correspondences_;
  uint64_t scan_version_ = 0;
  uint64_t map_increment_version_ = 0;
  uint64_t transformed_scan_version_ = 0;
  uint64_t correspondences_version_ = 0;
  uint64_t pose_version_ = 0;
};

} // namespace mslam