#pragma once

#include "mslam/common/Points.hh"
#include "mslam/config/IConfig.hh"
#include "mslam/map/Voxel.hh"

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <tsl/robin_map.h>
#include <tsl/robin_set.h>
#include <vector>

namespace mslam {

/// Why a keyframe was closed (sealed).
enum class CloseReason {
  Seed,     ///< the origin: sealed after the seed scans
  Distance, ///< the next scan moved more than keyframe_distance
  Angle,    ///< the next scan turned more than keyframe_angle
  Interval, ///< keyframe_max_interval_s passed
  Finish    ///< end of a recording (finish())
};

inline constexpr const char *toString(CloseReason reason) {
  switch (reason) {
  case CloseReason::Seed:
    return "seed";
  case CloseReason::Distance:
    return "distance";
  case CloseReason::Angle:
    return "angle";
  case CloseReason::Interval:
    return "interval";
  case CloseReason::Finish:
    return "finish";
  }
  return "unknown";
}

/// What made a scan open a keyframe.
enum class StartTrigger { First, Distance, Angle, Interval };

/// Decides when a scan opens a new keyframe: it moved more than `distance`
/// metres or turned more than `angle` radians from the current keyframe, or
/// (when `max_interval_s` > 0) that much time passed since the keyframe
/// opened. The scan that comes first always opens one.
class KeyframeSelector {
public:
  KeyframeSelector(double distance, double angle, double max_interval_s);

  bool hasKeyframe() const { return has_keyframe_; }
  /// True when the scan at (@p stamp_ns, @p pose) must open a new keyframe.
  /// @param use_interval False ignores `max_interval_s` (the seed scans).
  bool shouldStartKeyframe(std::int64_t stamp_ns, const Eigen::Affine3d &pose,
                           bool use_interval = true) const {
    return startTrigger(stamp_ns, pose, use_interval).has_value();
  }
  /// The reason the scan must open a keyframe, or nothing. Distance is checked
  /// before angle before interval, so that is the one reported when several
  /// hold.
  std::optional<StartTrigger> startTrigger(std::int64_t stamp_ns,
                                           const Eigen::Affine3d &pose,
                                           bool use_interval = true) const;
  void start(std::int64_t stamp_ns, const Eigen::Affine3d &pose);
  void reset() { has_keyframe_ = false; }

private:
  double distance_;
  double angle_;
  std::int64_t max_interval_ns_;
  bool has_keyframe_ = false;
  std::int64_t stamp_ns_ = 0;
  Eigen::Affine3d pose_ = Eigen::Affine3d::Identity();
};

/// Accumulates the scans of one keyframe into its frame, thinned.
///
/// Thinning is one running centroid per voxel of `voxel_size`: memory and the
/// cost of a scan are bounded by the occupied voxels however many scans pile
/// up, unlike re-running a VoxelFilter over the growing cloud (or a hash map
/// with a point budget per voxel, which keeps early points and gives no
/// averaged surface). Points come out in the order their voxel was first seen,
/// so the result is deterministic.
class MiniMapBuilder {
public:
  explicit MiniMapBuilder(double voxel_size);

  /// Starts an empty mini map in the frame of @p pose_odom (the keyframe's
  /// odom pose).
  void begin(const Eigen::Affine3d &pose_odom);
  /// Adds points given in the odom frame, as `T_odom_kf^-1 * p`.
  void add(const PointCloud &points_in_odom);
  /// Centroids so far (a copy: O(occupied voxels)).
  PointCloud snapshot() const;
  /// Returns the finished mini map and leaves the builder empty.
  std::shared_ptr<const PointCloud> seal();

  std::size_t scanCount() const { return scan_count_; }
  std::size_t voxelCount() const { return accumulators_.size(); }

private:
  struct Accumulator {
    double x = 0.0, y = 0.0, z = 0.0, intensity = 0.0;
    std::size_t count = 0;
  };

  double inverse_voxel_size_;
  Eigen::Affine3d odom_from_keyframe_ = Eigen::Affine3d::Identity();
  tsl::robin_map<Voxel3, std::uint32_t> slots_;
  std::vector<Accumulator> accumulators_;
  std::size_t scan_count_ = 0;
};

/// Decides which voxels a keyframe owns: a voxel belongs to the FIRST keyframe
/// that observes it, so the union of the keyframes' owned clouds has at most
/// `max_points_per_voxel` points per voxel however often a place is revisited.
///
/// With `max_points_per_voxel` 1 the keyframe keeps a running centroid per
/// voxel it owns. With more it keeps the first N raw points that fall into each
/// voxel (the same rule as `VoxelHashMap::addScan`), appended in the order they
/// arrive, so the sealed cloud is exactly the concatenation of the increments
/// `add()` returned. The open keyframe keeps filling its voxels until N;
/// sealing freezes them, partly filled or not.
///
/// Ownership is keyed by the voxel in the ODOM frame at `voxel_size`. The odom
/// frame never changes (the front end is self-consistent there and the back end
/// only corrects on the output side), so nothing needs re-keying after an
/// optimization. The sealed voxels are a hash set that only grows; the open
/// keyframe keeps what it owns per voxel (see above). It is used from the
/// caller thread only: the worker never sees it, only the clouds `seal()`
/// hands out.
class OwnedMapBuilder {
public:
  explicit OwnedMapBuilder(double voxel_size,
                           unsigned int max_points_per_voxel = 1);

  /// Starts the next keyframe (in the frame of @p pose_odom). The voxels owned
  /// so far stay owned.
  void begin(const Eigen::Affine3d &pose_odom);

  /// Adds a scan given in the odom frame. Points in voxels owned by a sealed
  /// keyframe are skipped, the others join the open keyframe's voxels.
  /// With N == 1 it returns, in the odom frame, one point (the mean of this
  /// scan's points in it) per voxel that THIS scan made the open keyframe own,
  /// in first-seen order; a voxel the open keyframe already owned is not
  /// returned again. With N > 1 it returns the points this scan added (up to
  /// N per voxel over the whole keyframe), in arrival order.
  PointCloud add(const PointCloud &points_in_odom);

  /// The open keyframe's owned points so far, in its own frame (a copy).
  PointCloud snapshot() const;

  /// Returns the open keyframe's owned points in its own frame (first-seen
  /// order at N == 1, arrival order at N > 1) and freezes its voxels: no later
  /// keyframe owns them.
  std::shared_ptr<const PointCloud> seal();

  /// Forgets everything, the sealed voxels included.
  void reset();

  std::size_t openVoxels() const {
    return max_points_per_voxel_ > 1 ? open_slots_.size() : open_.size();
  }
  std::size_t sealedVoxels() const { return sealed_.size(); }

private:
  struct Accumulator {
    double x = 0.0, y = 0.0, z = 0.0, intensity = 0.0;
    std::size_t count = 0;
  };

  PointCloud addCentroids(const PointCloud &points_in_odom);
  PointCloud addPoints(const PointCloud &points_in_odom);
  Voxel3 voxelOf(const Point &point) const;

  double inverse_voxel_size_;
  unsigned int max_points_per_voxel_;
  Eigen::Affine3d odom_from_keyframe_ = Eigen::Affine3d::Identity();
  tsl::robin_set<Voxel3> sealed_;
  // N == 1: voxel -> index into open_. N > 1: voxel -> points kept so far.
  tsl::robin_map<Voxel3, std::uint32_t> open_slots_;
  std::vector<Accumulator> open_;  // N == 1: odom frame, first-seen order
  std::vector<Accumulator> fresh_; // N == 1: this scan's points in new voxels
  PointCloud open_points_;         // N > 1: odom frame, arrival order
};

/// A keyframe whose clouds are final.
struct SealedKeyframe {
  std::int64_t stamp_ns = 0;
  std::size_t scan_count = 0;
  std::int64_t last_stamp_ns = 0; ///< time of the last scan it holds
  CloseReason close_reason = CloseReason::Finish;
  Eigen::Affine3d pose_odom = Eigen::Affine3d::Identity();
  /// Everything the keyframe saw (loop verification), keyframe frame.
  std::shared_ptr<const PointCloud> view;
  /// The voxels this keyframe owns (what gets published), keyframe frame.
  std::shared_ptr<const PointCloud> mini_map;
};

/// KeyframeSelector + MiniMapBuilder (the view) + OwnedMapBuilder (the owned
/// map). Fed one scan at a time; hands out the previous keyframe, exactly once,
/// when a scan opens the next one.
class KeyframeBuilder {
public:
  explicit KeyframeBuilder(const PoseGraphParameters &parameters);

  /// @param points_in_odom_frame The scan, already in the odom frame.
  /// @param seed_scan A seed scan (identity pose, before registration starts):
  ///        the interval trigger is off for it, so the seed scans are never
  ///        split into several keyframes (distance and angle cannot fire at a
  ///        constant pose).
  /// @param newly_owned If not null, receives (odom frame) what this scan added
  ///        to the open keyframe's owned voxels (see OwnedMapBuilder::add): the
  ///        scan's contribution to the single-layer map.
  /// @return The keyframe that this scan sealed, if it opened a new one.
  std::optional<SealedKeyframe> onScan(std::int64_t stamp_ns,
                                       const Eigen::Affine3d &T_odom_scan,
                                       const PointCloud &points_in_odom_frame,
                                       PointCloud *newly_owned = nullptr,
                                       bool seed_scan = false);

  /// Seals the open keyframe, if any (end of a recording). The next scan then
  /// opens a new keyframe.
  std::optional<SealedKeyframe>
  finish(CloseReason reason = CloseReason::Finish);

  bool hasOpenKeyframe() const { return selector_.hasKeyframe(); }
  /// Odom pose of the open keyframe; only meaningful when hasOpenKeyframe().
  const Eigen::Affine3d &openPoseOdom() const { return open_pose_odom_; }
  /// The open keyframe's owned voxels so far, in its own frame: what the
  /// output shows of it until it is sealed.
  PointCloud openMiniMap() const { return owned_.snapshot(); }
  /// Everything the open keyframe saw so far, in its own frame.
  PointCloud openView() const { return mini_map_.snapshot(); }
  std::size_t openScanCount() const { return mini_map_.scanCount(); }

  void reset();

private:
  SealedKeyframe sealOpen(CloseReason reason);

  KeyframeSelector selector_;
  MiniMapBuilder mini_map_; ///< the view
  OwnedMapBuilder owned_;
  Eigen::Affine3d open_pose_odom_ = Eigen::Affine3d::Identity();
  std::int64_t open_stamp_ns_ = 0;
  std::int64_t last_stamp_ns_ = 0; ///< the previous scan's time
};

} // namespace mslam
