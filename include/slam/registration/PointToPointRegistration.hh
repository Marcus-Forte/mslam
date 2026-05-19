#pragma once

#include "IRegistration.hh"

#include <Eigen/Dense>

namespace mslam {

class PointToPointRegistration : public IRegistration {
public:
  using IRegistration::IRegistration;
  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;

private:
  std::vector<Eigen::Vector3d> inputs_buffer_;
  VectorPoint3d map_points_buffer_;
};

} // namespace mslam
