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
  virtual void publishMap(const PointCloud2D &map) const = 0;

  virtual void publishPose(const Pose2D &pose) const = 0;
};
} // namespace mslam