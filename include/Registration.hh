#pragma once

#include "common/Points.hh"
#include "common/Pose.hh"
#include "map/IMap2D.hh"
#include <vector>
namespace mslam {

class Registration {
public:
  Pose2D Align(const Pose2D &pose, const IMap2D &map, const PointCloud2D &scan);

private:
};

} // namespace mslam