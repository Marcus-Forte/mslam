#pragma once

#include "imu/IImu.hh"
#include "lidar/ILidar.hh"

namespace mslam {

class ISlam {
public:
  virtual void ResetPose() = 0;
  virtual void Predict(const msensor::IMUData &imuData) = 0;
  virtual void Update(const msensor::Scan3D &lidarData) = 0;
};
} // namespace mslam