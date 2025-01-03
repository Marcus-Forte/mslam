#pragma once

#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/IMap2D.hh"
namespace mslam {

class Registration {
public:
  Pose2D Align(const Pose2D &pose, const IMap2D &map, const PointCloud2 &scan);

private:
};

} // namespace mslam