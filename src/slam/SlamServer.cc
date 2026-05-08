#include "slam/SlamServer.hh"

#include "conversion.hh"

#include <chrono>
#include <cstdlib>
#include <stdexcept>

namespace {

constexpr char g_default_slam_server_address[] = "0.0.0.0:50052";
constexpr int g_max_grpc_message_size = 64 * 1024 * 1024;

struct MapSnapshot {
  mslam::PointCloud3 map;
  uint64_t version = 0;
};

struct ScanSnapshot {
  mslam::PointCloud3 scan;
  uint64_t version = 0;
};

} // namespace

namespace mslam {

SlamServer::SlamServer(std::shared_ptr<ILog> logger, std::string address)
    : logger_(std::move(logger)),
      address_(address.empty() ? g_default_slam_server_address : address),
      pose_(Pose3D::Zero()) {}

SlamServer::~SlamServer() { stop(); }

void SlamServer::start() {
  if (server_) {
    return;
  }

  grpc::ServerBuilder builder;
  builder.SetMaxSendMessageSize(g_max_grpc_message_size);
  builder.SetMaxReceiveMessageSize(g_max_grpc_message_size);
  builder.AddListeningPort(address_, grpc::InsecureServerCredentials());
  builder.RegisterService(static_cast<sensors::SlamService::Service *>(this));
  builder.RegisterService(static_cast<sensors::ScanService::Service *>(this));

  server_ = builder.BuildAndStart();
  if (!server_) {
    throw std::runtime_error("Failed to start SLAM gRPC server");
  }

  if (logger_) {
    logger_->log(ILog::Level::INFO, "SLAM gRPC server listening on {}",
                 address_);
  }
}

void SlamServer::stop() {
  if (!server_) {
    return;
  }

  server_->Shutdown();
  server_.reset();
}

void SlamServer::updatePose(const Pose3D &pose) {
  std::scoped_lock lock(mutex_);
  pose_ = pose;
}

void SlamServer::updateMap(const PointCloud3 &map) {
  std::scoped_lock lock(mutex_);
  map_ = map;
  ++map_version_;
  map_cv_.notify_all();
}

void SlamServer::updateScan(const PointCloud3 &scan) {
  std::scoped_lock lock(mutex_);
  scan_ = scan;
  ++scan_version_;
  scan_cv_.notify_all();
}

grpc::Status
SlamServer::GetMap(grpc::ServerContext *context, const sensors::Empty *,
                   grpc::ServerWriter<sensors::PointCloud3> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled()) {
    MapSnapshot snapshot;
    {
      std::unique_lock lock(mutex_);
      map_cv_.wait_for(lock, std::chrono::milliseconds(100),
                       [&]() { return map_version_ != last_version; });
      if (map_version_ == last_version) {
        continue;
      }

      snapshot.map = map_;
      snapshot.version = map_version_;
    }

    last_version = snapshot.version;
    auto response = toGRPC(snapshot.map);
    if (!writer->Write(response)) {
      break;
    }
  }

  return grpc::Status::OK;
}

grpc::Status
SlamServer::GetScan(grpc::ServerContext *context, const sensors::Empty *,
                    grpc::ServerWriter<sensors::PointCloud3> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled()) {
    ScanSnapshot snapshot;
    {
      std::unique_lock lock(mutex_);
      scan_cv_.wait_for(lock, std::chrono::milliseconds(100),
                        [&]() { return scan_version_ != last_version; });
      if (scan_version_ == last_version) {
        continue;
      }

      snapshot.scan = scan_;
      snapshot.version = scan_version_;
    }

    last_version = snapshot.version;
    auto response = toGRPC(snapshot.scan);
    if (!writer->Write(response)) {
      break;
    }
  }

  return grpc::Status::OK;
}

grpc::Status SlamServer::GetPose(grpc::ServerContext *, const sensors::Empty *,
                                 sensors::Pose3D *response) {
  std::scoped_lock lock(mutex_);
  *response = toGRPC(pose_);
  return grpc::Status::OK;
}

} // namespace mslam