#pragma once

#include "IRegistration.hh"

namespace mslam {

class PointToPlaneRegistration : public IRegistration {
public:
  using IRegistration::IRegistration;
  SlamState Align(const SlamState &state, const IMap &map,
                  const PointCloud &scan) override;
};

} // namespace mslam
