#include "slam/SlamServer.hh"

#include "Conversion.hh"

#include <chrono>
#include <cstdlib>
#include <stdexcept>

namespace {

constexpr char g_default_slam_server_address[] = "0.0.0.0:50052";
constexpr int g_max_grpc_message_size = 64 * 1024 * 1024;

struct ScanSnapshot {
  mslam::PointCloud3 scan;
  uint64_t version = 0;
};

struct TransformedScanSnapshot {
  mslam::PointCloud3 scan;
  uint64_t version = 0;
};

struct CorrespondenceSnapshot {
  mslam::PointCloud3 correspondences;
  uint64_t version = 0;
};

struct PoseSnapshot {
  mslam::Pose3D pose;
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

  stopping_.store(false);

  grpc::ServerBuilder builder;
  builder.SetMaxSendMessageSize(g_max_grpc_message_size);
  builder.SetMaxReceiveMessageSize(g_max_grpc_message_size);
  builder.AddListeningPort(address_, grpc::InsecureServerCredentials());
  builder.RegisterService(static_cast<sensors::SlamService::Service *>(this));

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

  stopping_.store(true);
  scan_cv_.notify_all();
  transformed_scan_cv_.notify_all();
  correspondences_cv_.notify_all();
  pose_cv_.notify_all();

  server_->Shutdown();
  server_.reset();
}

void SlamServer::updatePose(const Pose3D &pose) {
  std::scoped_lock lock(mutex_);
  pose_ = pose;
  ++pose_version_;
  pose_cv_.notify_all();
}

void SlamServer::updateMap(const PointCloud3 &map) {
  std::scoped_lock lock(mutex_);
  map_ = map;
}

void SlamServer::updateScan(const PointCloud3 &scan) {
  std::scoped_lock lock(mutex_);
  scan_ = scan;
  ++scan_version_;
  scan_cv_.notify_all();
}

void SlamServer::updateTransformedScan(const PointCloud3 &scan) {
  std::scoped_lock lock(mutex_);
  transformed_scan_ = scan;
  ++transformed_scan_version_;
  transformed_scan_cv_.notify_all();
}

void SlamServer::updateCorrespondences(const PointCloud3 &correspondences) {
  std::scoped_lock lock(mutex_);
  correspondences_ = correspondences;
  ++correspondences_version_;
  correspondences_cv_.notify_all();
}

grpc::Status SlamServer::GetMap(grpc::ServerContext *, const sensors::Empty *,
                                sensors::PointCloud3 *response) {
  std::scoped_lock lock(mutex_);
  *response = toGRPC(map_);
  return grpc::Status::OK;
}

grpc::Status
SlamServer::GetScan(grpc::ServerContext *context, const sensors::Empty *,
                    grpc::ServerWriter<sensors::PointCloud3> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled() && !stopping_.load()) {
    ScanSnapshot snapshot;
    {
      std::unique_lock lock(mutex_);
      scan_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
        return stopping_.load() || scan_version_ != last_version;
      });
      if (stopping_.load()) {
        break;
      }
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

grpc::Status SlamServer::GetTransformedScan(
    grpc::ServerContext *context, const sensors::Empty *,
    grpc::ServerWriter<sensors::PointCloud3> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled() && !stopping_.load()) {
    TransformedScanSnapshot snapshot;
    {
      std::unique_lock lock(mutex_);
      transformed_scan_cv_.wait_for(
          lock, std::chrono::milliseconds(100), [&]() {
            return stopping_.load() ||
                   transformed_scan_version_ != last_version;
          });
      if (stopping_.load()) {
        break;
      }
      if (transformed_scan_version_ == last_version) {
        continue;
      }

      snapshot.scan = transformed_scan_;
      snapshot.version = transformed_scan_version_;
    }

    last_version = snapshot.version;
    auto response = toGRPC(snapshot.scan);
    if (!writer->Write(response)) {
      break;
    }
  }

  return grpc::Status::OK;
}

grpc::Status SlamServer::GetCorrespondences(
    grpc::ServerContext *context, const sensors::Empty *,
    grpc::ServerWriter<sensors::PointCloud3> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled() && !stopping_.load()) {
    CorrespondenceSnapshot snapshot;
    {
      std::unique_lock lock(mutex_);
      correspondences_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
        return stopping_.load() || correspondences_version_ != last_version;
      });
      if (stopping_.load()) {
        break;
      }
      if (correspondences_version_ == last_version) {
        continue;
      }

      snapshot.correspondences = correspondences_;
      snapshot.version = correspondences_version_;
    }

    last_version = snapshot.version;
    auto response = toGRPC(snapshot.correspondences);
    if (!writer->Write(response)) {
      break;
    }
  }

  return grpc::Status::OK;
}

grpc::Status SlamServer::GetPose(grpc::ServerContext *context,
                                 const sensors::Empty *,
                                 grpc::ServerWriter<sensors::Pose3D> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled() && !stopping_.load()) {
    PoseSnapshot snapshot;
    {
      std::unique_lock lock(mutex_);
      pose_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
        return stopping_.load() || pose_version_ != last_version;
      });
      if (stopping_.load()) {
        break;
      }
      if (pose_version_ == last_version) {
        continue;
      }

      snapshot.pose = pose_;
      snapshot.version = pose_version_;
    }

    last_version = snapshot.version;
    auto response = toGRPC(snapshot.pose);
    if (!writer->Write(response)) {
      break;
    }
  }

  return grpc::Status::OK;
}

} // namespace mslam