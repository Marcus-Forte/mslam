#pragma once

#include "mslam/common/Points.hh"
#include "mslam/common/State.hh"
#include "mslam/map/IMap.hh"

namespace mslam {

/// Scan-to-map registration: refines a pose estimate against a map.
class IRegistration {
public:
  virtual ~IRegistration() = default;

  /// @param state Initial guess.
  /// @param map   Reference map.
  /// @param scan  Scan in the sensor (body) frame.
  /// @return The refined state.
  virtual SlamState Align(const SlamState &state, const IMap &map,
                          const PointCloud &scan) = 0;
};

} // namespace mslam
