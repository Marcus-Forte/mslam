#include "mslam/slam/posegraph/PoseGraphBackend.hh"

#include "mslam/slam/Transform.hh"

#include <chrono>
#include <spdlog/fmt/fmt.h>
#include <string>

namespace mslam {

PoseGraphBackend::PoseGraphBackend(const PoseGraphParameters &parameters,
                                   std::shared_ptr<spdlog::logger> logger)
    : parameters_(parameters), logger_(logger), builder_(parameters),
      loop_closer_(parameters, logger),
      result_(std::make_shared<const PoseGraphResult>()) {
  if (!parameters_.synchronous) {
    worker_ = std::thread([this] { workerLoop(); });
  }
}

PoseGraphBackend::~PoseGraphBackend() {
  if (worker_.joinable()) {
    {
      const std::lock_guard lock(queue_mutex_);
      stop_ = true;
      queue_.clear(); // unprocessed keyframes are dropped
    }
    work_cv_.notify_all();
    worker_.join();
  }
}

PointCloud PoseGraphBackend::onScan(std::int64_t stamp_ns,
                                    const Eigen::Affine3d &T_odom_scan,
                                    const PointCloud &points_in_odom_frame,
                                    bool seed_scan) {
  PointCloud newly_owned;
  auto sealed = builder_.onScan(stamp_ns, T_odom_scan, points_in_odom_frame,
                                &newly_owned, seed_scan);
  if (sealed) {
    submit(std::move(*sealed));
  }
  return newly_owned;
}

void PoseGraphBackend::sealOpenKeyframe(CloseReason reason) {
  if (auto sealed = builder_.finish(reason)) {
    submit(std::move(*sealed));
  }
}

void PoseGraphBackend::finish() {
  sealOpenKeyframe();
  flush();
}

void PoseGraphBackend::submit(SealedKeyframe sealed) {
  ++sealed_count_;
  if (parameters_.synchronous) {
    process(std::move(sealed));
    return;
  }
  {
    const std::lock_guard lock(queue_mutex_);
    queue_.push_back(std::move(sealed));
  }
  work_cv_.notify_one();
}

void PoseGraphBackend::flush() {
  if (parameters_.synchronous) {
    return;
  }
  std::unique_lock lock(queue_mutex_);
  idle_cv_.wait(lock, [this] { return queue_.empty() && !busy_; });
}

void PoseGraphBackend::reset() {
  builder_.reset();
  sealed_count_ = 0;
  if (parameters_.synchronous) {
    graph_ = PoseGraph{};
    stats_ = PoseGraphStats{};
    rendered_.clear();
    rendered_poses_.clear();
    publish(false);
    return;
  }
  // The worker is idle once the queue is empty and busy_ is clear; it cannot
  // start new work while we hold the lock.
  std::unique_lock lock(queue_mutex_);
  queue_.clear();
  idle_cv_.wait(lock, [this] { return !busy_; });
  graph_ = PoseGraph{};
  stats_ = PoseGraphStats{};
  rendered_.clear();
  rendered_poses_.clear();
  publish(false);
}

std::shared_ptr<const PoseGraphResult> PoseGraphBackend::latestResult() const {
  const std::lock_guard lock(result_mutex_);
  return result_;
}

void PoseGraphBackend::workerLoop() {
  std::unique_lock lock(queue_mutex_);
  while (true) {
    work_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
    if (stop_) {
      return;
    }
    SealedKeyframe sealed = std::move(queue_.front());
    queue_.pop_front();
    busy_ = true;
    lock.unlock();
    process(std::move(sealed));
    lock.lock();
    busy_ = false;
    idle_cv_.notify_all();
  }
}

void PoseGraphBackend::process(SealedKeyframe sealed) {
  Keyframe keyframe;
  keyframe.id = graph_.keyframes.size();
  keyframe.stamp_ns = sealed.stamp_ns;
  keyframe.scan_count = sealed.scan_count;
  keyframe.pose_odom = sealed.pose_odom;
  keyframe.view = std::move(sealed.view);
  keyframe.mini_map = std::move(sealed.mini_map);

  if (keyframe.id == 0) {
    keyframe.pose = keyframe.pose_odom; // the gauge
    graph_.keyframes.push_back(std::move(keyframe));
  } else {
    const Keyframe &previous = graph_.keyframes.back();
    PoseEdge edge;
    edge.kind = EdgeKind::Odometry;
    edge.from = previous.id;
    edge.to = keyframe.id;
    edge.measured =
        Eigen::Affine3d(inverseRigid(previous.pose_odom) * keyframe.pose_odom);
    edge.information =
        informationFromSigmas(parameters_.odometry_sigma_translation,
                              parameters_.odometry_sigma_rotation);
    keyframe.pose = Eigen::Affine3d(previous.pose * edge.measured);
    graph_.keyframes.push_back(std::move(keyframe));
    graph_.edges.push_back(edge);
    ++stats_.odometry_edges;
  }
  stats_.keyframes = graph_.keyframes.size();
  if (graph_.keyframes.back().view) {
    stats_.view_points += graph_.keyframes.back().view->size();
  }
  if (graph_.keyframes.back().mini_map) {
    stats_.owned_points += graph_.keyframes.back().mini_map->size();
  }
  if (graph_.keyframes.size() == 1) {
    origin_stamp_ns_ = graph_.keyframes.back().stamp_ns;
  }
  if (logger_->should_log(spdlog::level::info)) {
    logKeyframe(graph_.keyframes.back(), sealed,
                graph_.keyframes.size() > 1 ? &graph_.edges.back() : nullptr);
  }

  bool optimized = false;
  const std::size_t id = graph_.keyframes.size() - 1;
  if (id > 0) {
    const LoopCloseResult loops = loop_closer_.close(graph_, id);
    for (const auto &attempt : loops.attempts) {
      attempt.accepted ? ++stats_.candidates_accepted
                       : ++stats_.candidates_rejected;
    }
    if (!loops.edges.empty()) {
      const std::size_t first_new_loop = graph_.edges.size();
      graph_.edges.insert(graph_.edges.end(), loops.edges.begin(),
                          loops.edges.end());
      stats_.loop_edges += loops.edges.size();

      // What the log line compares against: read before the optimizer moves
      // anything (these only read the graph).
      const bool report = logger_->should_log(spdlog::level::info);
      std::vector<EdgeResidual> loop_before;
      std::vector<Eigen::Affine3d> poses_before;
      Eigen::Affine3d map_odom_before = Eigen::Affine3d::Identity();
      if (report) {
        for (std::size_t e = first_new_loop; e < graph_.edges.size(); ++e) {
          loop_before.push_back(edgeResidualNorms(graph_, graph_.edges[e]));
        }
        for (const auto &kf : graph_.keyframes) {
          poses_before.push_back(kf.pose);
        }
        const Keyframe &last = graph_.keyframes.back();
        map_odom_before =
            Eigen::Affine3d(last.pose * inverseRigid(last.pose_odom));
      }

      OptimizerOptions options;
      options.max_iterations = parameters_.optimizer_iterations;
      const auto start = std::chrono::steady_clock::now();
      stats_.last_optimization = optimizePoseGraph(graph_, options);
      stats_.optimizer_seconds += std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() - start)
                                      .count();
      ++stats_.optimizations;
      optimized = true;
      if (report) {
        logOptimization(first_new_loop, loop_before, poses_before,
                        map_odom_before);
      }
    }
  }
  publish(optimized);
}

void PoseGraphBackend::logKeyframe(const Keyframe &keyframe,
                                   const SealedKeyframe &sealed,
                                   const PoseEdge *odometry_edge) const {
  const Eigen::Matrix3d rotation = keyframe.pose_odom.linear();
  const Eigen::Vector3d position = keyframe.pose_odom.translation();
  const Eigen::Vector3d rpy = rotationToEuler(rotation) * (180.0 / M_PI);

  // The map pose differs from the odom pose only once an optimization moved
  // T_map_odom away from the identity.
  std::string map = "map = odom";
  if (map_version_ > 0) {
    const Eigen::Matrix3d map_rotation = keyframe.pose.linear();
    const Eigen::Vector3d map_position = keyframe.pose.translation();
    const Eigen::Vector3d map_rpy =
        rotationToEuler(map_rotation) * (180.0 / M_PI);
    map = fmt::format("map x y z = {:.3f} {:.3f} {:.3f} m, rpy = {:.2f} "
                      "{:.2f} {:.2f} deg",
                      map_position.x(), map_position.y(), map_position.z(),
                      map_rpy.x(), map_rpy.y(), map_rpy.z());
  }

  std::string from_previous;
  if (odometry_edge != nullptr) {
    const Eigen::Matrix3d step = odometry_edge->measured.linear();
    const Eigen::Vector3d moved = odometry_edge->measured.translation();
    from_previous = fmt::format(
        " | from kf {}: {:.2f} m {:.2f} deg", odometry_edge->from, moved.norm(),
        Eigen::AngleAxisd(step).angle() * (180.0 / M_PI));
  }

  logger_->info(
      "keyframe {}/{} @ {:.2f} s ({} scans, {:.2f} s, closed by {}) | odom x y "
      "z = {:.3f} {:.3f} {:.3f} m, rpy = {:.2f} {:.2f} {:.2f} deg, {}{} | "
      "owned {} pts (total {}), view {} pts",
      keyframe.id, graph_.keyframes.size(),
      1e-9 * static_cast<double>(keyframe.stamp_ns - origin_stamp_ns_),
      keyframe.scan_count,
      1e-9 * static_cast<double>(sealed.last_stamp_ns - keyframe.stamp_ns),
      toString(sealed.close_reason), position.x(), position.y(), position.z(),
      rpy.x(), rpy.y(), rpy.z(), map, from_previous,
      keyframe.mini_map ? keyframe.mini_map->size() : 0, stats_.owned_points,
      keyframe.view ? keyframe.view->size() : 0);
}

void PoseGraphBackend::logOptimization(
    std::size_t first_new_loop, const std::vector<EdgeResidual> &loop_before,
    const std::vector<Eigen::Affine3d> &poses_before,
    const Eigen::Affine3d &map_odom_before) const {
  const OptimizerResult &result = stats_.last_optimization;
  const std::size_t edges = graph_.edges.size();

  // Line 1: the cost, and the new loop edge(s) before and after.
  std::string loops;
  for (std::size_t e = first_new_loop; e < edges; ++e) {
    const PoseEdge &edge = graph_.edges[e];
    const EdgeResidual before = loop_before[e - first_new_loop];
    const EdgeResidual after = edgeResidualNorms(graph_, edge);
    loops += fmt::format(
        " | loop {}->{}: {:.1f} mm {:.2f} mrad -> {:.2f} mm {:.3f} mrad "
        "(verify rms {:.1f} mm, matched {:.0f} %)",
        edge.from, edge.to, 1e3 * before.translation_m,
        1e3 * before.rotation_rad, 1e3 * after.translation_m,
        1e3 * after.rotation_rad, 1e3 * edge.verify_rms,
        100.0 * edge.verify_matched_fraction);
  }
  logger_->info("pose graph: {} keyframes, {} edges, {} in {} iterations, cost "
                "{:.4g} -> {:.4g}, chi2/dof {:.3g} -> {:.3g}{}",
                graph_.keyframes.size(), edges,
                result.converged ? "converged" : "NOT converged",
                result.iterations, result.initial_cost, result.final_cost,
                normalizedChiSquared(result.initial_cost, edges),
                normalizedChiSquared(result.final_cost, edges), loops);

  // Line 2: every edge after the optimization, and what it moved.
  const GraphErrorStats errors = graphErrorStats(graph_);
  std::string worst = "none";
  if (errors.worst_edge != GraphErrorStats::kNoEdge) {
    const PoseEdge &edge = graph_.edges[errors.worst_edge];
    worst = fmt::format("{}->{} {} ({:.1f} sigma)", edge.from, edge.to,
                        edge.kind == EdgeKind::Loop ? "loop" : "odometry",
                        errors.worst_sigmas);
  }
  const LargestPoseChange moved = largestPoseChange(poses_before, graph_);
  const Keyframe &last = graph_.keyframes.back();
  const PoseDelta map_odom =
      poseDelta(map_odom_before,
                Eigen::Affine3d(last.pose * inverseRigid(last.pose_odom)));
  const auto group = [](const EdgeGroupStats &g) {
    return fmt::format("max/rms {:.1f}/{:.1f} mm {:.2f}/{:.2f} mrad (n={})",
                       1e3 * g.max_translation_m, 1e3 * g.rms_translation_m,
                       1e3 * g.max_rotation_rad, 1e3 * g.rms_rotation_rad,
                       g.count);
  };
  logger_->info(
      "pose graph residuals after: odometry {} | loops {} | worst {} | "
      "moved: keyframe {} by {:.1f} mm, keyframe {} by {:.2f} mrad, "
      "T_map_odom by {:.1f} mm {:.2f} mrad",
      group(errors.odometry), group(errors.loop), worst,
      moved.translation_keyframe, 1e3 * moved.max.translation_m,
      moved.rotation_keyframe, 1e3 * moved.max.rotation_rad,
      1e3 * map_odom.translation_m, 1e3 * map_odom.rotation_rad);
}

void PoseGraphBackend::publish(bool optimized) {
  auto result = std::make_shared<PoseGraphResult>();
  result->graph_version = ++version_;
  if (optimized) {
    ++map_version_;
  }
  result->map_version = map_version_;
  result->stats = stats_;
  result->edges = graph_.edges;

  const std::size_t count = graph_.keyframes.size();
  result->poses.reserve(count);
  result->map_clouds.reserve(count);
  result->mini_maps.reserve(count);
  rendered_.resize(count);
  rendered_poses_.resize(count);
  for (std::size_t k = 0; k < count; ++k) {
    const Keyframe &keyframe = graph_.keyframes[k];
    result->poses.push_back(keyframe.pose);
    result->stamps_ns.push_back(keyframe.stamp_ns);
    result->mini_maps.push_back(keyframe.mini_map);
    // Render again only the keyframes that moved (exact comparison: an
    // unmoved pose is bit-identical).
    if (!rendered_[k] ||
        !rendered_poses_[k].matrix().isApprox(keyframe.pose.matrix(), 0.0)) {
      PointCloud cloud = keyframe.mini_map ? *keyframe.mini_map : PointCloud{};
      transformCloud(keyframe.pose, cloud);
      rendered_[k] = std::make_shared<const PointCloud>(std::move(cloud));
      rendered_poses_[k] = keyframe.pose;
    }
    result->map_clouds.push_back(rendered_[k]);
  }
  if (count > 0 && map_version_ > 0) {
    const Keyframe &last = graph_.keyframes.back();
    result->T_map_odom =
        Eigen::Affine3d(last.pose * inverseRigid(last.pose_odom));
  }

  const std::lock_guard lock(result_mutex_);
  result_ = std::move(result);
}

} // namespace mslam
