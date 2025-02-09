#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/IMap.hh"

namespace mslam {

class Registration {
public:
  Registration(int num_registration_iterations_, int num_optimizer_iterations_,
               float max_correspondence_distance,
               const std::shared_ptr<ILog> &logger);
  /**
   * @brief
   *
   * @param pose input pose in 2 dimensions. Parameter vector represents x, y
   * and theta.
   * @param map
   * @param scan
   * @return Pose2D
   */
  Pose2D Align2D(const Pose2D &pose, const IMap &map, const PointCloud3 &scan);

  /**
   * @brief
   *
   * @param pose input pose in 3 dimensions. Parameter vector represents x, y,
   * z, roll, pitch and yaw.
   * @param map
   * @param scan
   * @return Pose2D
   */
  Pose3D Align3D(const Pose3D &pose, const IMap &map, const PointCloud3 &scan);

private:
  int num_registration_iterations_;
  int num_optimizer_iterations_;
  float max_correspondence_distance_;
  std::shared_ptr<ILog> logger_;
};

} // namespace mslam