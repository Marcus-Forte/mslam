#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "mslam/config/IConfig.hh"

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace mslam {

/// What the pose-graph back end reports for the status RPC.
struct GraphStatus {
  SlamStrategy strategy = SlamStrategy::Odometry;
  std::uint64_t graph_version = 0;
  std::size_t keyframes = 0;
  std::size_t loop_closures = 0; ///< accepted loop edges
};

/// Where the SLAM pipeline publishes its results. Implemented by the gRPC
/// server; tests and benchmarks use their own or NullSlamOutput. All calls
/// come from the SLAM thread.
class ISlamOutput {
public:
  virtual ~ISlamOutput() = default;

  virtual void updatePose(const Pose3D &pose) = 0;

  /// The latest scan, transformed into the map frame. Ownership is handed over.
  virtual void updateTransformedScan(PointCloud scan) = 0;

  /// The points the last scan added to the map. Ownership is handed over.
  virtual void updateMapIncrement(PointCloud increment) = 0;

  /// A map increment that is already shared (the pose-graph back end re-sends
  /// its rendered keyframe clouds this way). The default copies it into the
  /// owning overload; an output that stores shared clouds should override this
  /// to keep the pointer, which makes a re-send O(1) per chunk.
  virtual void updateMapIncrement(std::shared_ptr<const PointCloud> increment) {
    updateMapIncrement(PointCloud(*increment));
  }

  /// The pose-graph back end published a new result. Never called in the
  /// odometry strategy. The default ignores it.
  virtual void updateGraphStatus(const GraphStatus &) {}

  /// The SLAM state was reset: anything accumulated from earlier increments
  /// (map and keyframes) is stale.
  virtual void reset() = 0;

  /// Only the accumulated map is stale: the pose-graph back end is about to
  /// re-send it (after an optimization moved the keyframes). Keyframes sent
  /// through addKeyframe() stay valid. The default is a full reset().
  virtual void clearMap() { reset(); }

  /// Pose-graph strategy: keyframe @p id (consecutive from 0 since the last
  /// reset()) with its mini map in the keyframe's own frame and its pose in the
  /// map frame. Sent once per keyframe. Never called for odometry.
  virtual void addKeyframe(std::uint64_t /*id*/,
                           const Eigen::Affine3d & /*pose*/,
                           std::shared_ptr<const PointCloud> /*cloud*/) {}

  /// Pose-graph strategy: an optimization moved the keyframes; @p poses holds
  /// the map-frame pose of keyframes [0, poses.size()). Never called for
  /// odometry.
  virtual void updateKeyframePoses(std::uint64_t /*graph_version*/,
                                   std::span<const Eigen::Affine3d> /*poses*/) {
  }
};

/// Discards everything.
class NullSlamOutput final : public ISlamOutput {
public:
  void updatePose(const Pose3D &) override {}
  void updateTransformedScan(PointCloud) override {}
  using ISlamOutput::updateMapIncrement;
  void updateMapIncrement(PointCloud) override {}
  void reset() override {}
};

/// Control of a running SLAM loop, callable from any thread (the gRPC
/// server's handlers). Requests are applied by the SLAM thread between two
/// scans.
class ISlamControl {
public:
  virtual ~ISlamControl() = default;

  virtual void startProcessing() = 0;
  virtual void stopProcessing() = 0;

  /// Stops processing, then clears the pose, IMU state and map before the
  /// next scan is handled.
  virtual void requestReset() = 0;
};

} // namespace mslam
