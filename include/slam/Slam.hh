#pragma once

#include "ILog.hh"
#include "ISlam.hh"
#include "config/IConfig.hh"
#include "map/IMap.hh"
#include "slam/Preprocessor.hh"
#include "slam/Registration.hh"
#include <atomic>

namespace mslam {

class SlamServer;
class PointCloudExporter;
class RecordingSensorPlayer;

class Slam : public ISlam {
public:
  Slam(const std::shared_ptr<ILog> &logger, const SlamConfiguration &config,
       const std::shared_ptr<IMap> &map);
  void ResetPose() override;
  void Predict(const msensor::IMUData &imuData) override;
  void Update(const msensor::Scan3D &lidarData) override;
  Pose3D getPose() const;
  Eigen::Affine3d getTransform() const;
  const VectorPoint3d &getLastMapCorrespondences() const;

  void run(std::shared_ptr<msensor::ILidar> lidar,
           std::shared_ptr<msensor::IImu> imu, SlamServer &server,
           PointCloudExporter &exporter,
           std::shared_ptr<RecordingSensorPlayer> playback_player = {});

  void startProcessing();
  void stopProcessing();
  void reset();
  bool isRunning() const;

private:
  static void signalHandler(int signal_number);
  static std::atomic<bool> should_stop_;

  SlamConfiguration config_;
  std::shared_ptr<IMap> map_;
  Preprocessor preprocessor_;
  Registration registration_;
  Pose3D pose_;
  std::shared_ptr<ILog> logger_;
  std::atomic<bool> running_{true};
};
} // namespace mslam