#pragma once

#include "common/Pose.hh"
#include "map/IMap2D.hh"
namespace mslam {

class Registration {
public:
  Pose2D Align(const Pose2D &pose, IMap2D *map, const PointCloud2D &scan);

private:
};

} // namespace mslam