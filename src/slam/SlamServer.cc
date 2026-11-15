#include "mslam/slam/SlamServer.hh"

#include "mslam/slam/Conversion.hh"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <random>
#include <stdexcept>

namespace {

constexpr char g_default_slam_server_address[] = "0.0.0.0:50052";
constexpr int g_max_grpc_message_size = 64 * 1024 * 1024;

struct TransformedScanSnapshot {
  std::shared_ptr<const mslam::PointCloud> scan;
  uint64_t version = 0;
};

struct CorrespondenceSnapshot {
  mslam::PointCloud correspondences;
  uint64_t version = 0;
};

struct PoseSnapshot {
  mslam::Pose3D pose;
  uint64_t version = 0;
};

// StreamMap limits. The cap keeps a chunk (16 bytes per point) well under
// gRPC's default 4 MiB client limit.
constexpr uint32_t g_default_points_per_chunk = 65536;
constexpr uint32_t g_max_points_per_chunk = 200000;
// Chunk pointers a stream copies per lock acquisition, so the lock hold time
// does not grow with the backlog.
constexpr size_t g_max_pointers_per_lock = 1024;

// Non-zero and different between processes.
uint64_t initialEpoch() {
  std::random_device device;
  uint64_t epoch = (static_cast<uint64_t>(device()) << 32) ^ device();
  epoch ^= static_cast<uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
  return epoch == 0 ? 1 : epoch;
}

} // namespace

namespace mslam {

SlamServer::SlamServer(std::shared_ptr<spdlog::logger> logger,
                       std::string address)
    : logger_(std::move(logger)),
      address_(address.empty() ? g_default_slam_server_address : address),
      map_epoch_(initialEpoch()), pose_(Pose3D::Zero()) {}

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
    logger_->info("SLAM gRPC server listening on {}", address_);
  }
}

void SlamServer::stop() {
  if (!server_) {
    return;
  }

  stopping_.store(true);
  map_increment_cv_.notify_all();
  transformed_scan_cv_.notify_all();
  correspondences_cv_.notify_all();
  pose_cv_.notify_all();

  server_->Shutdown();
  server_.reset();
}

void SlamServer::updatePose(const Pose3D &pose) {
  std::scoped_lock lock(pose_mutex_);
  pose_ = pose;
  ++pose_version_;
  pose_cv_.notify_all();
}

void SlamServer::updateMapIncrement(PointCloud increment) {
  auto shared_increment =
      std::make_shared<const PointCloud>(std::move(increment));
  std::scoped_lock lock(map_increment_mutex_);
  map_increment_ = shared_increment;
  // Retain every increment so GetMap can serve the full accumulated map even
  // though the registration map is bounded. Storing a shared handle keeps the
  // per-scan cost independent of the map size.
  map_history_.emplace_back(std::move(shared_increment));
  ++map_increment_version_;
  map_increment_cv_.notify_all();
}

void SlamServer::updateTransformedScan(PointCloud scan) {
  auto shared_scan = std::make_shared<const PointCloud>(std::move(scan));
  std::scoped_lock lock(transformed_scan_mutex_);
  transformed_scan_ = std::move(shared_scan);
  ++transformed_scan_version_;
  transformed_scan_cv_.notify_all();
}

void SlamServer::reset() {
  {
    std::scoped_lock lock(map_increment_mutex_);
    map_increment_.reset();
    map_history_.clear();
    // O(1): open StreamMap streams see the new epoch the next time they wake.
    if (++map_epoch_ == 0) {
      map_epoch_ = 1;
    }
    ++map_increment_version_;
  }
  map_increment_cv_.notify_all();
  // Pose version bumps come from updatePose(); the transformed scan and
  // correspondences are replaced by the next scan.
}

void SlamServer::updateCorrespondences(const PointCloud &correspondences) {
  std::scoped_lock lock(correspondences_mutex_);
  correspondences_ = correspondences;
  ++correspondences_version_;
  correspondences_cv_.notify_all();
}

grpc::Status SlamServer::GetMap(grpc::ServerContext *, const sensors::Empty *,
                                sensors::PointCloud3 *response) {
  // Copy the pointer list in bounded slices so the SLAM thread never waits
  // long on the mutex; start over if a reset intervenes.
  std::vector<std::shared_ptr<const PointCloud>> history;
  for (;;) {
    history.clear();
    uint64_t epoch = 0;
    bool complete = false;
    while (!complete) {
      std::scoped_lock lock(map_increment_mutex_);
      if (history.empty()) {
        epoch = map_epoch_;
      } else if (epoch != map_epoch_) {
        break;
      }
      const auto end = std::min(map_history_.size(),
                                history.size() + g_max_pointers_per_lock);
      history.insert(history.end(), map_history_.begin() + history.size(),
                     map_history_.begin() + end);
      complete = end == map_history_.size();
    }
    if (complete) {
      break;
    }
  }

  size_t total_points = 0;
  for (const auto &chunk : history) {
    total_points += chunk->size();
  }

  PointCloud full_map;
  full_map.reserve(total_points);
  for (const auto &chunk : history) {
    full_map.insert(full_map.end(), chunk->begin(), chunk->end());
  }

  *response = toGRPC(full_map);
  return grpc::Status::OK;
}

grpc::Status
SlamServer::StreamMap(grpc::ServerContext *context,
                      const sensors::MapStreamRequest *request,
                      grpc::ServerWriter<sensors::MapStreamChunk> *writer) {
  size_t max_points = request->max_points_per_chunk();
  if (max_points == 0) {
    max_points = g_default_points_per_chunk;
  }
  max_points = std::min<size_t>(max_points, g_max_points_per_chunk);

  // The stream's cursor: `offset` is the first increment not yet sent, and
  // `pending` holds the increments [offset, offset + pending.size()) already
  // copied out of the history (pointers only).
  uint64_t epoch = 0;
  uint64_t offset = 0;
  std::vector<std::shared_ptr<const PointCloud>> pending;

  auto write_empty = [&]() {
    sensors::MapStreamChunk chunk;
    chunk.set_epoch(epoch);
    chunk.set_first_offset(offset);
    chunk.set_next_offset(offset);
    return writer->Write(chunk);
  };

  {
    std::scoped_lock lock(map_increment_mutex_);
    epoch = map_epoch_;
    if (request->epoch() == epoch) {
      offset = std::min<uint64_t>(request->start_offset(), map_history_.size());
    }
  }
  if (!write_empty()) {
    return grpc::Status::OK;
  }

  while (!context->IsCancelled() && !stopping_.load()) {
    bool reset_seen = false;
    {
      std::unique_lock lock(map_increment_mutex_);
      auto available = [&]() {
        return offset + pending.size() < map_history_.size();
      };
      map_increment_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
        return stopping_.load() || map_epoch_ != epoch || !pending.empty() ||
               available();
      });
      if (stopping_.load()) {
        break;
      }
      if (map_epoch_ != epoch) {
        epoch = map_epoch_;
        offset = 0;
        pending.clear();
        reset_seen = true;
      } else {
        // Top up the local queue, at most g_max_pointers_per_lock pointers
        // per acquisition.
        const size_t next = offset + pending.size();
        const size_t count =
            std::min(map_history_.size() - std::min(next, map_history_.size()),
                     g_max_pointers_per_lock -
                         std::min(pending.size(), g_max_pointers_per_lock));
        pending.insert(pending.end(), map_history_.begin() + next,
                       map_history_.begin() + next + count);
      }
    }
    if (reset_seen) {
      // Announce the new epoch at once, before any of its data.
      if (!write_empty()) {
        break;
      }
      continue;
    }
    if (pending.empty()) {
      continue;
    }

    // Concatenate whole increments up to the bound (an oversized one goes
    // alone). Empty increments are absorbed into the range.
    PointCloud points;
    const uint64_t first_offset = offset;
    size_t consumed = 0;
    while (consumed < pending.size()) {
      const auto &increment = *pending[consumed];
      if (!points.empty() && points.size() + increment.size() > max_points) {
        break;
      }
      points.insert(points.end(), increment.begin(), increment.end());
      ++consumed;
    }
    pending.erase(pending.begin(),
                  pending.begin() + static_cast<std::ptrdiff_t>(consumed));
    offset += consumed;
    if (points.empty()) {
      continue; // only empty increments: the offset advanced, nothing to send
    }

    sensors::MapStreamChunk chunk;
    chunk.set_epoch(epoch);
    chunk.set_first_offset(first_offset);
    chunk.set_next_offset(offset);
    *chunk.mutable_points() = toGRPC(points);
    if (!writer->Write(chunk)) {
      break;
    }
  }

  return grpc::Status::OK;
}

grpc::Status
SlamServer::GetMapIncrements(grpc::ServerContext *context,
                             const sensors::Empty *,
                             grpc::ServerWriter<sensors::PointCloud3> *writer) {
  uint64_t last_version = 0;
  while (!context->IsCancelled() && !stopping_.load()) {
    std::shared_ptr<const PointCloud> snapshot;
    {
      std::unique_lock lock(map_increment_mutex_);
      map_increment_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
        return stopping_.load() || map_increment_version_ != last_version;
      });
      if (stopping_.load()) {
        break;
      }
      if (map_increment_version_ == last_version) {
        continue;
      }

      snapshot = map_increment_;
      last_version = map_increment_version_;
    }
    if (!snapshot) {
      continue;
    }

    auto response = toGRPC(*snapshot);
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
      std::unique_lock lock(transformed_scan_mutex_);
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
    auto response = toGRPC(*snapshot.scan);
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
      std::unique_lock lock(correspondences_mutex_);
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
      std::unique_lock lock(pose_mutex_);
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

void SlamServer::setControl(ISlamControl *control) { control_.store(control); }

grpc::Status SlamServer::Stop(grpc::ServerContext *, const sensors::Empty *,
                              sensors::Empty *) {
  if (auto *control = control_.load()) {
    control->stopProcessing();
  }
  return grpc::Status::OK;
}

grpc::Status SlamServer::Start(grpc::ServerContext *, const sensors::Empty *,
                               sensors::Empty *) {
  if (auto *control = control_.load()) {
    control->startProcessing();
  }
  return grpc::Status::OK;
}

// The reset itself, including the map history this server keeps, is carried
// out by the SLAM thread between two scans (it ends in reset() above), so it
// cannot race with a scan being processed.
grpc::Status SlamServer::Reset(grpc::ServerContext *, const sensors::Empty *,
                               sensors::Empty *) {
  if (auto *control = control_.load()) {
    control->requestReset();
  }
  return grpc::Status::OK;
}

} // namespace mslam
