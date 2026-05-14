#pragma once

#include "IRegistration.hh"

namespace mslam {

class PointToPointRegistration : public IRegistration {
public:
  using IRegistration::IRegistration;
  Pose3D Align(const Pose3D &pose, const IMap &map,
               const PointCloud &scan) override;
};

} // namespace mslam
