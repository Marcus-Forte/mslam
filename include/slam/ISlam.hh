#pragma once

#include "common/Points.hh"
#include "msensor/interface/IImu.hh"

namespace mslam {

class ISlam {
public:
  virtual void ResetPose() = 0;
  /**
   * @brief Predict the next pose based on IMU data.
   *
   * @param imuData
   */
  virtual void Predict(const msensor::IMUData &imuData) = 0;

  /**
   * @brief Correct the pose estimate based on a new LiDAR scan.
   *
   * @param lidarData
   */
  virtual void Update(const Scan &lidarData) = 0;
};
} // namespace mslam