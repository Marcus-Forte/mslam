#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "slam.pb.h"

namespace mslam {

sensors::Pose3D toGRPC(const Pose3D &pose);
sensors::PointCloud3 toGRPC(const PointCloud &map);

} // namespace mslam