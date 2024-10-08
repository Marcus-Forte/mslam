#pragma once

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
};

} // namespace mslam