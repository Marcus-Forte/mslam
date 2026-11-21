#pragma once

#include "mslam/config/IConfig.hh"
#include "mslam/map/IMap.hh"
#include "mslam/slam/ImuScanSync.hh"
#include "mslam/slam/Preprocessor.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/posegraph/PoseGraphBackend.hh"

#include <chrono>
#include <memory>
#include <spdlog/logger.h>
#include <vector>

namespace mslam {

/// Wall-clock time spent in each stage of the pipeline, accumulated over all
/// the processed scans.
struct StageTimings {
  std::chrono::nanoseconds predict{};    ///< IMU dead reckoning (all samples)
  std::chrono::nanoseconds preprocess{}; ///< deskew + filters
  std::chrono::nanoseconds update{};     ///< scan-to-map registration
  std::chrono::nanoseconds map_insert{}; ///< transform + map insertion
  /// Pose-graph strategy only: SLAM-thread time in the back end (handing it the
  /// scan, reading its result, re-sending the map).
  std::chrono::nanoseconds backend{};
  std::size_t map_resends = 0; ///< times the whole map was re-sent
  std::size_t scans = 0;       ///< scans processed
};

/**
 * @brief The synchronous per-scan SLAM sequence, independent of where the
 * sensor data comes from.
 *
 * For one scan: predict with the IMU samples of the scan's time window,
 * preprocess the scan, register it against the map (Slam::Update), insert it
 * into the map(s) and publish the pose, the transformed scan and the map
 * increment. The first `kInitScans` scans only seed the map. The live
 * SlamRunner and the offline benchmark both drive this class, so they run the
 * same code.
 *
 * IMU samples and scans are paired by timestamp (ImuScanSynchronizer), not by
 * arrival: feed every sample to pushImu(), hold a scan until imuCovers() is
 * true (or give up waiting), then call processSyncedScan(). See docs/arch.md
 * for the time convention.
 *
 * Not thread-safe: use it from one thread.
 */
class SlamPipeline {
public:
  /// Scans that only seed the map before registration starts.
  static constexpr int kInitScans = 10;

  /// @param slam The estimator; its map is the registration map.
  SlamPipeline(Slam &slam, const SlamConfiguration &config,
               std::shared_ptr<spdlog::logger> logger, ISlamOutput &output);

  /// Queues an IMU sample for the scan whose window it falls in. Ignored when
  /// the IMU is disabled.
  void pushImu(const msensor::IMUData &imu);

  /// True if the IMU stream has reached the end of @p scan, i.e. the scan's
  /// window is complete. Always true when the IMU is disabled or the
  /// preprocessor has no `points_per_second` (no scan end to wait for).
  bool imuCovers(const Scan &scan) const { return sync_.covers(scan); }

  /// Processes one scan with its own IMU window: integrates the queued
  /// samples with `timestamp <= t_end` (plus the split sample that lands the
  /// state exactly on `t_end`), then registers the scan. Call it when
  /// imuCovers() holds; if the caller gave up waiting it still works, with the
  /// samples that have arrived (the estimator then falls back to LiDAR-only
  /// registration if none did).
  void processSyncedScan(const Scan &scan);

  /// Processes one IMU sample (prediction) and publishes the pose. Low-level:
  /// bypasses the synchroniser.
  void processImu(const msensor::IMUData &imu);

  /// Processes one scan. @p imu_samples are the samples already given to
  /// processImu() for this scan's window; deskewing uses them. Low-level:
  /// bypasses the synchroniser.
  void processScan(const Scan &scan,
                   const std::vector<msensor::IMUData> &imu_samples);

  /// Resets the pose, the IMU state and the maps (and the pose-graph back
  /// end), and tells the output.
  void reset();

  /// End of a recording (pose-graph strategy; a no-op for odometry): seals the
  /// open keyframe, waits for the back end to drain and publishes its final
  /// result, re-sending the map as the keyframe clouds of that result. Call it
  /// from the SLAM thread.
  void finish();

  /// The back end's latest result; null for the odometry strategy.
  std::shared_ptr<const PoseGraphResult> backendResult() const {
    return backend_ ? backend_->latestResult() : nullptr;
  }
  /// Correction of the odom frame into the map frame (identity for odometry).
  const Eigen::Affine3d &mapFromOdom() const { return T_map_odom_; }

  /// Publishes the current pose to the output.
  void publishPose();

  const StageTimings &timings() const { return timings_; }
  const ImuSyncStats &syncStats() const { return sync_.stats(); }

private:
  void initScan(const Scan &scan);
  /// Adds the points (already in the map frame) to the maps and publishes
  /// them. @p points is consumed: ownership goes to the output. @p owned
  /// (pose-graph strategy; odom frame, consumed) is what is published as the
  /// map increment instead of the registration map's own increment, so the
  /// published map has each voxel once.
  void insertAndPublish(PointCloud &points, PointCloud *owned = nullptr);
  /// Bound the registration map around the current pose.
  void pruneMap();
  /// Pose-graph strategy: takes the back end's latest result, keeps its
  /// T_map_odom and re-sends the map when an optimization made it stale.
  void applyBackendResult(bool force_resend = false);
  void resendMap(const PoseGraphResult &result);
  /// The pose the output gets: the odom pose, corrected by T_map_odom.
  Pose3D outputPose() const;

  Slam &slam_;
  SlamConfiguration config_;
  std::shared_ptr<spdlog::logger> logger_;
  ISlamOutput &output_;
  std::shared_ptr<IMap> map_;
  std::unique_ptr<IMap> dense_map_;
  Preprocessor preprocessor_;
  ImuScanSynchronizer sync_;
  ImuScanSynchronizer::Window window_;
  Scan filtered_scan_;
  int init_scan_count_ = 0;
  std::uint64_t last_imu_time_ns_ = 0;
  StageTimings timings_;

  // Pose-graph strategy (backend_ is null for odometry, and then none of the
  // rest is used).
  std::unique_ptr<PoseGraphBackend> backend_;
  std::shared_ptr<const PoseGraphResult> applied_result_;
  Eigen::Affine3d T_map_odom_ = Eigen::Affine3d::Identity();
  bool corrected_ = false; ///< T_map_odom_ is not the identity
  std::uint64_t published_map_version_ = 0;
  /// Keyframes whose cloud the output holds since the last re-send, and how
  /// many had been sealed then (the worker may still have been behind: their
  /// scans were wiped by the re-send and come back with later results).
  std::size_t resent_keyframes_ = 0;
  std::size_t resent_sealed_ = 0;
  /// Keyframes given to the output with addKeyframe() since the last reset, and
  /// the map_version whose poses it was last given.
  std::size_t keyframes_sent_ = 0;
  std::uint64_t keyframe_poses_map_version_ = 0;
};

} // namespace mslam
