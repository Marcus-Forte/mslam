#pragma once

#include "mslam/common/Points.hh"
#include "mslam/config/IConfig.hh"
#include "mslam/slam/posegraph/KeyframeBuilder.hh"
#include "mslam/slam/posegraph/LoopCloser.hh"
#include "mslam/slam/posegraph/PoseGraph.hh"
#include "mslam/slam/posegraph/PoseGraphOptimizer.hh"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <spdlog/logger.h>
#include <thread>
#include <vector>

namespace mslam {

struct PoseGraphStats {
  std::size_t keyframes = 0;
  std::size_t odometry_edges = 0;
  std::size_t loop_edges = 0;
  std::size_t candidates_accepted = 0;
  std::size_t candidates_rejected = 0;
  std::size_t optimizations = 0;
  /// Points owned (published) by all keyframes so far.
  std::size_t owned_points = 0;
  /// Points held in the keyframes' views (what the loop verification matches
  /// against, everything each keyframe saw), summed over the keyframes.
  std::size_t view_points = 0;
  double optimizer_seconds = 0.0; ///< Wall time, not part of the determinism.
  OptimizerResult last_optimization;
};

/// One consistent snapshot of the back end, immutable once published.
struct PoseGraphResult {
  /// Increases with every published change (a new keyframe or an
  /// optimization). Not reset by reset(): it only grows.
  std::uint64_t graph_version = 0;
  /// Increases only when an optimization moved the existing keyframes, i.e.
  /// when a map built from earlier results is stale.
  std::uint64_t map_version = 0;
  /// Correction of the odom frame into the map frame:
  /// `pose_kf * pose_kf_odom^-1` of the newest keyframe. Exactly the identity
  /// when empty and until the first optimization (before it every keyframe pose
  /// is just the chained odometry).
  Eigen::Affine3d T_map_odom = Eigen::Affine3d::Identity();
  std::vector<Eigen::Affine3d> poses;  ///< Optimized pose per keyframe.
  std::vector<std::int64_t> stamps_ns; ///< Opening scan time per keyframe.
  std::vector<PoseEdge> edges;
  /// `pose * mini_map` per keyframe (map frame). A cloud is shared with the
  /// previous result when its keyframe did not move.
  std::vector<std::shared_ptr<const PointCloud>> map_clouds;
  /// Each keyframe's owned mini map in its own frame (what a viewer places with
  /// the keyframe's pose; a voxel belongs to the first keyframe that saw it).
  /// Never changes once sealed.
  std::vector<std::shared_ptr<const PointCloud>> mini_maps;
  PoseGraphStats stats;
};

/// The pose-graph back end: keyframes and mini maps from the odom-frame scans,
/// odometry and loop edges, optimization when a loop closes.
///
/// Threading: onScan(), flush() and reset() belong to one caller thread (the
/// SLAM thread); latestResult() may be called from any thread. With
/// `synchronous` the whole back end runs inside onScan(); otherwise sealed
/// keyframes go through a queue to one worker thread that closes loops,
/// optimizes and renders, and the caller only does the cheap mini-map
/// insertion. Both modes process the same keyframes in the same order, so the
/// graph after flush() is identical.
class PoseGraphBackend {
public:
  PoseGraphBackend(const PoseGraphParameters &parameters,
                   std::shared_ptr<spdlog::logger> logger);
  ~PoseGraphBackend();
  PoseGraphBackend(const PoseGraphBackend &) = delete;
  PoseGraphBackend &operator=(const PoseGraphBackend &) = delete;

  /// @param points_in_odom_frame The scan, already in the odom frame.
  /// @param seed_scan One of the pipeline's seed scans: the keyframe interval
  ///        trigger is ignored, so they all land in the origin keyframe.
  /// @return The points (odom frame) of the voxels this scan made the open
  ///         keyframe own: the scan's contribution to the single-layer map.
  ///         Ownership is decided here, on the caller thread; the worker only
  ///         ever sees sealed clouds.
  PointCloud onScan(std::int64_t stamp_ns, const Eigen::Affine3d &T_odom_scan,
                    const PointCloud &points_in_odom_frame,
                    bool seed_scan = false);

  /// Seals the open keyframe without waiting for the worker: it joins the
  /// graph when processed (at once when synchronous), and the next onScan()
  /// opens a new keyframe. The pipeline uses it to make the seed scans the
  /// origin keyframe. finish() is this plus a drain.
  /// @param reason Why it closes, for the keyframe log line.
  void sealOpenKeyframe(CloseReason reason = CloseReason::Finish);

  /// Blocks until every sealed keyframe is processed and published. No-op when
  /// synchronous.
  void flush();

  /// End of a recording: seals the open keyframe (it joins the graph and may
  /// close a loop), then flush(). A later onScan() opens a new keyframe.
  void finish();

  /// Keyframes sealed so far, including those still queued for the worker
  /// (the result only holds the processed ones). Caller thread only.
  std::size_t sealedKeyframes() const { return sealed_count_; }

  /// Drops everything (queued work, graph, open mini map) and publishes an
  /// empty result with a higher graph_version.
  void reset();

  /// Never null.
  std::shared_ptr<const PoseGraphResult> latestResult() const;

  /// The open (unsealed) keyframe, for publishing the newest points: its odom
  /// pose and its mini map in its own frame. Caller thread only.
  bool hasOpenKeyframe() const { return builder_.hasOpenKeyframe(); }
  const Eigen::Affine3d &openKeyframePoseOdom() const {
    return builder_.openPoseOdom();
  }
  PointCloud openMiniMap() const { return builder_.openMiniMap(); }

private:
  void submit(SealedKeyframe sealed);
  void process(SealedKeyframe sealed);
  /// The info line of a keyframe entering the graph.
  void logKeyframe(const Keyframe &keyframe, const SealedKeyframe &sealed,
                   const PoseEdge *odometry_edge) const;
  /// The two info lines of an optimization: cost, new loop edges before and
  /// after, all edges after, and how far the optimization moved the poses.
  void logOptimization(std::size_t first_new_loop,
                       const std::vector<EdgeResidual> &loop_before,
                       const std::vector<Eigen::Affine3d> &poses_before,
                       const Eigen::Affine3d &map_odom_before) const;
  void publish(bool optimized);
  void workerLoop();

  PoseGraphParameters parameters_;
  std::shared_ptr<spdlog::logger> logger_;
  KeyframeBuilder builder_;      // caller thread only
  std::size_t sealed_count_ = 0; // caller thread only

  // Owned by whoever runs process(): the caller (synchronous) or the worker.
  PoseGraph graph_;
  LoopCloser loop_closer_;
  PoseGraphStats stats_;
  std::vector<std::shared_ptr<const PointCloud>> rendered_;
  std::vector<Eigen::Affine3d> rendered_poses_;
  std::uint64_t version_ = 0;
  std::uint64_t map_version_ = 0;
  std::int64_t origin_stamp_ns_ = 0; ///< keyframe 0's time, for the log

  mutable std::mutex result_mutex_;
  std::shared_ptr<const PoseGraphResult> result_;

  // Worker queue.
  std::mutex queue_mutex_;
  std::condition_variable work_cv_;
  std::condition_variable idle_cv_;
  std::deque<SealedKeyframe> queue_;
  bool busy_ = false;
  bool stop_ = false;
  std::thread worker_;
};

} // namespace mslam
