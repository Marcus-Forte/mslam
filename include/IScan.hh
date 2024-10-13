#pragma once

#include "common//IMU.hh"
#include "common/Points.hh"
namespace mslam {

class IScan {
public:
  /**
   * @brief Get one scan from sensor.
   *
   * @param blocking blocking call.
   * @return PointCloud2T timestamped pointcloud.
   */
  virtual PointCloud2T getScan(bool blocking = false) = 0;

  virtual IMUData getImuData() = 0;
};

} // namespace mslam