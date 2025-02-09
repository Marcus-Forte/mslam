#pragma once

#include "ILog.hh"
#include "ISlam.hh"
#include "config/IConfig.hh"
#include "map/IMap.hh"
#include "slam/Registration.hh"

namespace mslam {

class Slam : public ISlam {
public:
  Slam(const std::shared_ptr<ILog> &logger, const SlamParameters &config,
       const std::shared_ptr<IMap> &map);
  void ResetPose() override;
  void Predict(const msensor::IMUData &imuData) override;
  void Update(const msensor::Scan3D &lidarData) override;
  Pose3D getPose() const;

private:
  SlamParameters config_;
  std::shared_ptr<IMap> map_;

  Registration registration_;
  Pose3D pose_;
  std::shared_ptr<ILog> logger_;
};
} // namespace mslam