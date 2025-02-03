#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/IMap.hh"

namespace mslam {

class Registration2D {
public:
  Registration2D(int num_registration_iterations_,
                 int num_optimizer_iterations_,
                 float max_correspondence_distance,
                 const std::shared_ptr<ILog> &logger);
  /**
   * @brief
   *
   * @param pose represents x, y and theta.
   * @param map
   * @param scan
   * @return Pose2D
   */
  Pose2D Align(const Pose2D &pose, const IMap &map, const PointCloud3 &scan);

private:
  int num_registration_iterations_;
  int num_optimizer_iterations_;
  float max_correspondence_distance_;
  std::shared_ptr<ILog> logger_;
};

} // namespace mslam