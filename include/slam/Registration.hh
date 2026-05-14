#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "common/Pose.hh"
#include "config/IConfig.hh"
#include "map/IMap.hh"

namespace mslam {

class Registration {
public:
  using RegistrationCallback2D =
      std::function<void(const Pose2D &pose, const VectorPoint2d &map_corrs,
                         const VectorPoint2d &src_corrs)>;
  Registration(int num_registration_iterations_, int num_optimizer_iterations_,
               float max_correspondence_distance,
               const std::shared_ptr<ILog> &logger);
  /**
   * @brief
   *
   * @param pose input pose in 2 dimensions. Parameter vector represents x, y. z
   * cordinates are ignored. and theta.
   * @param map
   * @param scan
   * @return Pose2D
   */
  Pose2D Align2D(const Pose2D &pose, const IMap &map, const PointCloud &scan);

  /**
   * @brief
   *
   * @param pose input pose in 3 dimensions. Parameter vector represents x, y,
   * z, roll, pitch and yaw.
   * @param map
   * @param scan
   * @param metric 3D registration metric to use for optimization.
   * @return Pose
   */
  Pose3D Align3D(const Pose3D &pose, const IMap &map, const PointCloud &scan,
                 RegistrationMetric3D metric);

  /**
   * @brief Register a callback function that will be called after each
   * iteration of the registration process.
   */
  void registerIterationCallback2D(RegistrationCallback2D &&callback);

  const VectorPoint3d &getLastMapCorrespondences() const {
    return last_map_correspondences_;
  }

private:
  int num_registration_iterations_;
  int num_optimizer_iterations_;
  float max_correspondence_distance_;
  std::shared_ptr<ILog> logger_;
  VectorPoint3d last_map_correspondences_;

  RegistrationCallback2D registration_callback2d_;
};

} // namespace mslam