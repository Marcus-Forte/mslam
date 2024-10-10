#pragma once

#include "common//IMU.hh"
#include "common/Points.hh"
namespace mslam {

class IScan {
public:
  /**
   * @brief Gets one scan from sensor.
   *
   * @param blocking blocking call.
   * @return PointCloud2D
   */
  virtual PointCloud2D getScan(bool blocking = false) = 0;

  virtual IMUData getImuData() = 0;
};

} // namespace mslam