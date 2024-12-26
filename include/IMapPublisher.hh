#pragma once

#include "common/Points.hh"
#include "common/Pose.hh"

namespace mslam {
class IMapPublisher {
public:
  /**
   * @brief Publish map
   *
   * @param map
   */

  virtual void publishScan(const PointCloud2 &map, float r, float g, float b,
                           const std::string &name) const = 0;

  virtual void publishPose(const Pose2D &pose) const = 0;
};
} // namespace mslam