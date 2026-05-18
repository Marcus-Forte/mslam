#pragma once

#include "ILog.hh"
#include "ISlam.hh"
#include "config/IConfig.hh"
#include "map/IMap.hh"
#include "slam/ImuPreintegration.hh"
#include "slam/registration/IRegistration.hh"
#include "slam/registration/ImuRegistration.hh"

#include <Eigen/Dense>
#include <atomic>
#include <optional>

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
  void Update(const Scan &lidarData) override;
  Pose3D getPose() const;
  Eigen::Affine3d getTransform() const;

  void run(std::shared_ptr<msensor::ILidar> lidar,
           std::shared_ptr<msensor::IImu> imu, SlamServer &server,
           PointCloudExporter &exporter,
           std::shared_ptr<RecordingSensorPlayer> playback_player = {});

  void startProcessing();
  void stopProcessing();
  void reset();
  bool isRunning() const;

private:
  void ResetImuPreintegration();
  bool TryInitializeGravityAlignment(const msensor::IMUData &imuData);

  static void signalHandler(int signal_number);
  static std::atomic<bool> should_stop_;

  SlamConfiguration config_;
  std::shared_ptr<IMap> map_;
  std::unique_ptr<IMap> dense_map_;
  std::unique_ptr<IRegistration> registration_;
  std::unique_ptr<ImuRegistration> imu_registration_;
  SlamState state_;
  std::optional<uint64_t> last_imu_timestamp_ns_;
  bool imu_gravity_aligned_ = false;
  std::shared_ptr<ILog> logger_;
  std::atomic<bool> running_{true};

  // IMU preintegration state
  ImuPreintegrator preintegrator_{ImuNoiseParams{}};
  SlamState previous_state_;
  bool has_previous_state_ = false;
};
} // namespace mslam