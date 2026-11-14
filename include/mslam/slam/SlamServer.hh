#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "mslam/slam/SlamOutput.hh"
#include "slam.grpc.pb.h"
#include <atomic>
#include <condition_variable>
#include <grpcpp/grpcpp.h>
#include <memory>
#include <mutex>
#include <spdlog/logger.h>
#include <string>

namespace mslam {

/// gRPC front end: streams the SLAM output (as the pipeline's ISlamOutput) to
/// clients and forwards their start / stop / reset requests to an
/// ISlamControl.
class SlamServer : public sensors::SlamService::Service, public ISlamOutput {
public:
  explicit SlamServer(std::shared_ptr<spdlog::logger> logger,
                      std::string address = {});
  ~SlamServer();

  SlamServer(const SlamServer &) = delete;
  SlamServer &operator=(const SlamServer &) = delete;

  void start();
  void stop();

  /// The control the Start / Stop / Reset RPCs act on (not owned; must outlive
  /// the server or be cleared with nullptr before it is destroyed).
  void setControl(ISlamControl *control);

  // ISlamOutput. Called from the SLAM thread.
  void updatePose(const Pose3D &pose) override;
  void updateMapIncrement(PointCloud increment) override;
  void updateTransformedScan(PointCloud scan) override;
  /// Drops the accumulated map history and the latest increment.
  void reset() override;

  void updateCorrespondences(const PointCloud &correspondences);

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

  grpc::Status Stop(grpc::ServerContext *, const sensors::Empty *,
                    sensors::Empty *response) override;

  grpc::Status Start(grpc::ServerContext *, const sensors::Empty *,
                     sensors::Empty *response) override;

  grpc::Status Reset(grpc::ServerContext *, const sensors::Empty *,
                     sensors::Empty *response) override;

  std::shared_ptr<spdlog::logger> logger_;
  std::string address_;
  std::unique_ptr<grpc::Server> server_;
  std::atomic<bool> stopping_{false};
  std::atomic<ISlamControl *> control_{nullptr};

  mutable std::mutex map_increment_mutex_;
  mutable std::mutex transformed_scan_mutex_;
  mutable std::mutex correspondences_mutex_;
  mutable std::mutex pose_mutex_;

  mutable std::condition_variable map_increment_cv_;
  mutable std::condition_variable transformed_scan_cv_;
  mutable std::condition_variable correspondences_cv_;
  mutable std::condition_variable pose_cv_;
  Pose3D pose_;
  // The latest increment is the same object as the newest history entry: one
  // allocation per scan, shared by the streaming RPC and GetMap.
  std::shared_ptr<const PointCloud> map_increment_;
  // Full map history for GetMap. The SLAM registration map is bounded, so the
  // client-facing snapshot is accumulated here from the published increments
  // (pointers only, to keep the SLAM thread's cost O(1)).
  std::vector<std::shared_ptr<const PointCloud>> map_history_;
  std::shared_ptr<const PointCloud> transformed_scan_;
  PointCloud correspondences_;
  uint64_t scan_version_ = 0;
  uint64_t map_increment_version_ = 0;
  uint64_t transformed_scan_version_ = 0;
  uint64_t correspondences_version_ = 0;
  uint64_t pose_version_ = 0;
};

} // namespace mslam