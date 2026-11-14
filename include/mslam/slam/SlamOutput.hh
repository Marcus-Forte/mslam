#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"

namespace mslam {

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

  /// The SLAM state was reset: anything accumulated from earlier increments
  /// is stale.
  virtual void reset() = 0;
};

/// Discards everything.
class NullSlamOutput final : public ISlamOutput {
public:
  void updatePose(const Pose3D &) override {}
  void updateTransformedScan(PointCloud) override {}
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
