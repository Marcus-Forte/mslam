#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "mslam/slam/SensorStatus.hh"
#include "mslam/slam/SlamOutput.hh"
#include "slam.grpc.pb.h"
#include <atomic>
#include <condition_variable>
#include <grpcpp/grpcpp.h>
#include <memory>
#include <mutex>
#include <span>
#include <spdlog/logger.h>
#include <string>
#include <vector>

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

  /// Where the GetStatus RPC reads the sensor link from. Shared, so a request
  /// in flight never outlives it; nullptr detaches (GetStatus then reports
  /// SENSOR_LINK_UNKNOWN).
  void setSensorStatus(std::shared_ptr<const ISensorStatus> source);

  // ISlamOutput. Called from the SLAM thread.
  void updatePose(const Pose3D &pose) override;
  void updateMapIncrement(PointCloud increment) override;
  /// Keeps the pointer: O(1) on the SLAM thread whatever the cloud's size.
  void updateMapIncrement(std::shared_ptr<const PointCloud> increment) override;
  void updateGraphStatus(const GraphStatus &status) override;
  void updateTransformedScan(PointCloud scan) override;
  /// Full reset: drops the map (see clearMap()) and the keyframes, and starts
  /// a new keyframe epoch (open StreamKeyframes streams announce it).
  void reset() override;
  /// Drops the accumulated map history and the latest increment, and starts a
  /// new map epoch (open StreamMap streams announce it). The keyframes stay.
  void clearMap() override;
  /// O(1): the pointer is kept in the keyframe list.
  void addKeyframe(std::uint64_t id, const Eigen::Affine3d &pose,
                   std::shared_ptr<const PointCloud> cloud) override;
  /// The pose table is converted outside the lock and published by swapping a
  /// pointer. Streams send the latest table, skipping intermediate ones.
  void updateKeyframePoses(std::uint64_t graph_version,
                           std::span<const Eigen::Affine3d> poses) override;

  void updateCorrespondences(const PointCloud &correspondences);

private:
  grpc::Status GetMap(grpc::ServerContext *, const sensors::Empty *,
                      sensors::PointCloud3 *response) override;

  grpc::Status
  GetMapIncrements(grpc::ServerContext *context, const sensors::Empty *,
                   grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  grpc::Status
  StreamMap(grpc::ServerContext *context,
            const sensors::MapStreamRequest *request,
            grpc::ServerWriter<sensors::MapStreamChunk> *writer) override;

  grpc::Status StreamKeyframes(
      grpc::ServerContext *context,
      const sensors::KeyframeStreamRequest *request,
      grpc::ServerWriter<sensors::KeyframeStreamMessage> *writer) override;

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

  grpc::Status GetStatus(grpc::ServerContext *, const sensors::Empty *,
                         sensors::SlamStatus *response) override;

  std::shared_ptr<spdlog::logger> logger_;
  std::string address_;
  std::unique_ptr<grpc::Server> server_;
  std::atomic<bool> stopping_{false};
  std::atomic<ISlamControl *> control_{nullptr};
  mutable std::mutex sensor_status_mutex_;
  std::shared_ptr<const ISensorStatus> sensor_status_;

  // Keyframes (pose-graph strategy). Own epoch, not the map's: the map epoch
  // also changes whenever an optimization makes the pipeline re-send the point
  // map, while the keyframes and their clouds stay valid then. Only reset()
  // bumps keyframe_epoch_. Everything below is guarded by keyframe_mutex_.
  struct StoredKeyframe {
    std::shared_ptr<const PointCloud> cloud; // keyframe frame
    sensors::PoseQuat pose;                  // map frame, when it was added
  };
  struct PoseTable {
    uint64_t graph_version = 0;
    std::vector<sensors::PoseQuat> poses; // index = keyframe id
  };
  mutable std::mutex keyframe_mutex_;
  mutable std::condition_variable keyframe_cv_;
  uint64_t keyframe_epoch_;
  std::vector<StoredKeyframe> keyframes_; // index = keyframe id
  std::shared_ptr<const PoseTable> pose_table_;

  mutable std::mutex graph_status_mutex_;
  GraphStatus graph_status_;

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
  // The index in this vector is the increment's offset in StreamMap; offsets
  // restart at 0 in each epoch.
  std::vector<std::shared_ptr<const PointCloud>> map_history_;
  // Generation of map_history_ (guarded by map_increment_mutex_). Never 0,
  // seeded randomly per process, incremented by reset().
  uint64_t map_epoch_;
  std::shared_ptr<const PointCloud> transformed_scan_;
  PointCloud correspondences_;
  uint64_t scan_version_ = 0;
  uint64_t map_increment_version_ = 0;
  uint64_t transformed_scan_version_ = 0;
  uint64_t correspondences_version_ = 0;
  uint64_t pose_version_ = 0;
};

} // namespace mslam