#include "mslam/slam/SlamPipeline.hh"

#include <algorithm>

#include "mslam/slam/Transform.hh"

#include <utility>

namespace mslam {

namespace {

using Clock = std::chrono::steady_clock;

std::chrono::nanoseconds since(Clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                              start);
}

// Time the back end gets for a scan: the registration time, the scan's last
// point (docs/arch.md, "Time convention"); the first point's time when the
// scan has no per-point timing.
std::int64_t registrationTimeNs(const Scan &scan, unsigned points_per_second) {
  const auto end =
      ImuScanSynchronizer::scanEndTimestamp(scan, points_per_second);
  return static_cast<std::int64_t>(end.value_or(scan.header.timestamp));
}

} // namespace

SlamPipeline::SlamPipeline(Slam &slam, const SlamConfiguration &config,
                           std::shared_ptr<spdlog::logger> logger,
                           ISlamOutput &output)
    : slam_(slam), config_(config), logger_(std::move(logger)), output_(output),
      map_(slam.map()), preprocessor_(config.preprocessor, logger_),
      sync_(config.with_imu, config.preprocessor.points_per_second, logger_) {
  if (config_.strategy == SlamStrategy::PoseGraph) {
    backend_ = std::make_unique<PoseGraphBackend>(config_.pose_graph, logger_);
    applied_result_ = backend_->latestResult();
    GraphStatus status;
    status.strategy = SlamStrategy::PoseGraph;
    output_.updateGraphStatus(status);
  }
  logger_->debug("Using downsample filter: {}",
                 toString(config_.preprocessor.downsample_filter));
}

Pose3D SlamPipeline::outputPose() const {
  if (!corrected_) {
    return slam_.getPose();
  }
  const Eigen::Affine3d pose = T_map_odom_ * slam_.getTransform();
  const Eigen::Matrix3d rotation = pose.linear();
  const Eigen::Vector3d translation = pose.translation();
  const Eigen::Vector3d euler = rotationToEuler(rotation);
  Pose3D out;
  out << translation.x(), translation.y(), translation.z(), euler.x(),
      euler.y(), euler.z();
  return out;
}

void SlamPipeline::publishPose() { output_.updatePose(outputPose()); }

void SlamPipeline::pushImu(const msensor::IMUData &imu) { sync_.push(imu); }

void SlamPipeline::processSyncedScan(const Scan &scan) {
  sync_.take(scan, window_);
  for (const auto &imu : window_.samples) {
    processImu(imu);
  }
  if (window_.split.has_value()) {
    processImu(*window_.split);
  }
  processScan(scan, window_.samples);
}

void SlamPipeline::processImu(const msensor::IMUData &imu) {
  logger_->debug("Processing IMU @ {}, delta {} ms, seq nr {}, "
                 "raw_acc=[{}, {}, {}], raw_gyro=[{}, {}, {}]",
                 imu.header.timestamp,
                 (imu.header.timestamp - last_imu_time_ns_) * 1e-6,
                 imu.header.sequence_number, imu.ax, imu.ay, imu.az, imu.gx,
                 imu.gy, imu.gz);
  last_imu_time_ns_ = imu.header.timestamp;

  const auto start = Clock::now();
  slam_.Predict(imu);
  timings_.predict += since(start);
  publishPose();
}

void SlamPipeline::processScan(
    const Scan &scan, const std::vector<msensor::IMUData> &imu_samples) {
  ++timings_.scans;

  if (init_scan_count_ < kInitScans) {
    initScan(scan);
    return;
  }
  if (!config_.with_lidar) {
    return;
  }

  if (backend_) {
    const auto backend_start = Clock::now();
    applyBackendResult();
    timings_.backend += since(backend_start);
  }

  logger_->debug("Processing Lidar scan with {} points @ {}, seq nr {}",
                 scan.points.size(), scan.header.timestamp,
                 scan.header.sequence_number);

  auto start = Clock::now();
  preprocessor_.process(scan, imu_samples, filtered_scan_);
  timings_.preprocess += since(start);
  logger_->debug("Preprocess: {} -> {} pts", scan.points.size(),
                 filtered_scan_.points.size());

  start = Clock::now();
  slam_.Update(filtered_scan_);
  timings_.update += since(start);

  publishPose();

  start = Clock::now();
  std::chrono::nanoseconds backend_elapsed{};
  PointCloud owned;
  transformCloud(slam_.getTransform(), filtered_scan_.points);
  if (backend_) {
    // The points are in the odom frame now and insertAndPublish() consumes
    // them. The back end builds its mini maps from them.
    const auto backend_start = Clock::now();
    owned = backend_->onScan(
        registrationTimeNs(scan, config_.preprocessor.points_per_second),
        slam_.getTransform(), filtered_scan_.points);
    backend_elapsed = since(backend_start);
    timings_.backend += backend_elapsed;
  }
  insertAndPublish(filtered_scan_.points, backend_ ? &owned : nullptr);
  timings_.map_insert += since(start) - backend_elapsed;
}

void SlamPipeline::initScan(const Scan &scan) {
  const auto start = Clock::now();
  auto filtered_scan = preprocessor_.filterNearCenter(scan);
  timings_.preprocess += since(start);

  PointCloud owned;
  if (backend_) {
    // Init scans sit in the sensor frame at the identity pose: they form the
    // first keyframe.
    const auto backend_start = Clock::now();
    owned = backend_->onScan(
        registrationTimeNs(scan, config_.preprocessor.points_per_second),
        Eigen::Affine3d::Identity(), filtered_scan->points,
        /*seed_scan=*/true);
    timings_.backend += since(backend_start);
  }
  const auto insert_start = Clock::now();
  insertAndPublish(filtered_scan->points, backend_ ? &owned : nullptr);
  timings_.map_insert += since(insert_start);
  ++init_scan_count_;

  if (backend_ && init_scan_count_ == kInitScans) {
    // The seed scans, at the identity pose, are the origin keyframe: the
    // gauge, in the graph (and on screen) before the first registered scan.
    // The next scan opens keyframe 1. The result reaches applyBackendResult()
    // at the start of the next scan (at once when synchronous, a little later
    // with the worker); keyframes_sent_ makes sure it is published once.
    const auto backend_start = Clock::now();
    backend_->sealOpenKeyframe(CloseReason::Seed);
    timings_.backend += since(backend_start);
  }

  if (logger_->should_log(spdlog::level::info)) {
    logger_->info("Init scan {}/{}. Map points: {}", init_scan_count_,
                  kInitScans, map_->getPointCloudRepresentation().size());
  }
  slam_.ResetImuPreintegration();
}

void SlamPipeline::insertAndPublish(PointCloud &points, PointCloud *owned) {
  auto map_increment = map_->addScan(points);
  pruneMap();

  if (owned != nullptr) {
    // Pose-graph strategy: publish only what this scan added to the open
    // keyframe's owned voxels (odom frame), not the registration map's
    // increment, so the map the viewers accumulate has each voxel owned once
    // and at most max_points_per_voxel points in it.
    map_increment = std::move(*owned);
  }

  if (corrected_) {
    // The registration map stays in the odom frame; what is published is in
    // the map frame.
    transformCloud(T_map_odom_, points);
    transformCloud(T_map_odom_, map_increment);
  }
  output_.updateTransformedScan(std::move(points));
  output_.updateMapIncrement(std::move(map_increment));
}

void SlamPipeline::pruneMap() {
  const auto pose = slam_.getPose();
  map_->prune(Point{static_cast<float>(pose[0]), static_cast<float>(pose[1]),
                    static_cast<float>(pose[2])},
              config_.map_parameters.max_range,
              config_.map_parameters.max_voxels);
}

void SlamPipeline::applyBackendResult(bool force_resend) {
  auto result = backend_->latestResult();
  if (result == applied_result_ && !force_resend) {
    return;
  }
  applied_result_ = result;
  T_map_odom_ = result->T_map_odom;
  corrected_ = !T_map_odom_.matrix().isIdentity(0.0);

  if (force_resend || result->map_version != published_map_version_) {
    resendMap(*result);
  } else if (resent_keyframes_ < resent_sealed_ &&
             result->map_clouds.size() > resent_keyframes_) {
    // The worker was behind at the last re-send: those keyframes' scans were
    // wiped with the old map, so give their clouds now.
    const std::size_t end = std::min(result->map_clouds.size(), resent_sealed_);
    for (; resent_keyframes_ < end; ++resent_keyframes_) {
      output_.updateMapIncrement(result->map_clouds[resent_keyframes_]);
    }
  }

  // Keyframe-aware outputs: each new keyframe once (its mini map in its own
  // frame), then only poses when an optimization moved them.
  for (; keyframes_sent_ < result->poses.size(); ++keyframes_sent_) {
    output_.addKeyframe(keyframes_sent_, result->poses[keyframes_sent_],
                        result->mini_maps[keyframes_sent_]);
  }
  if (result->map_version != keyframe_poses_map_version_) {
    keyframe_poses_map_version_ = result->map_version;
    output_.updateKeyframePoses(result->graph_version, result->poses);
  }

  GraphStatus status;
  status.strategy = SlamStrategy::PoseGraph;
  status.graph_version = result->graph_version;
  status.keyframes = result->stats.keyframes;
  status.loop_closures = result->stats.loop_edges;
  output_.updateGraphStatus(status);
}

void SlamPipeline::resendMap(const PoseGraphResult &result) {
  ++timings_.map_resends;
  published_map_version_ = result.map_version;
  output_.clearMap(); // keyframes stay valid: only the point map is re-sent
  for (const auto &cloud : result.map_clouds) {
    if (cloud && !cloud->empty()) {
      output_.updateMapIncrement(cloud);
    }
  }
  resent_keyframes_ = result.map_clouds.size();
  resent_sealed_ = backend_->sealedKeyframes();
  if (backend_->hasOpenKeyframe()) {
    PointCloud open = backend_->openMiniMap();
    transformCloud(
        Eigen::Affine3d(T_map_odom_ * backend_->openKeyframePoseOdom()), open);
    if (!open.empty()) {
      output_.updateMapIncrement(std::move(open));
    }
  }
}

void SlamPipeline::finish() {
  if (!backend_) {
    return;
  }
  const auto start = Clock::now();
  backend_->finish();
  // Everything is a keyframe now: re-send the map as the final keyframe clouds.
  applyBackendResult(true);
  timings_.backend += since(start);
}

void SlamPipeline::reset() {
  sync_.clear();
  slam_.ResetPose();
  map_->clear();
  if (backend_) {
    backend_->reset();
    applied_result_ = backend_->latestResult();
    T_map_odom_ = Eigen::Affine3d::Identity();
    corrected_ = false;
    published_map_version_ = applied_result_->map_version;
    resent_keyframes_ = 0;
    resent_sealed_ = 0;
    keyframes_sent_ = 0;
    keyframe_poses_map_version_ = applied_result_->map_version;
    GraphStatus status;
    status.strategy = SlamStrategy::PoseGraph;
    status.graph_version = applied_result_->graph_version;
    output_.updateGraphStatus(status);
  }
  output_.reset();
  publishPose();
  logger_->info("SLAM reset: pose and map cleared");
}

} // namespace mslam
