#pragma once

#include "common/Points.hh"
#include "msensor/interface/IImu.hh"

namespace mslam {

class ISlam {
public:
  virtual void ResetPose() = 0;
  virtual void Predict(const msensor::IMUData &imuData) = 0;
  virtual void Update(const Scan &lidarData) = 0;
};
} // namespace mslam