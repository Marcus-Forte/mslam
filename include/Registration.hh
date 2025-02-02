#pragma once

#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/IMap.hh"

namespace mslam {

class Registration {
public:
  Pose2D Align(const Pose2D &pose, const IMap &map, const PointCloud3 &scan);

private:
};

} // namespace mslam