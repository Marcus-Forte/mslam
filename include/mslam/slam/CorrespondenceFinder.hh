#pragma once

#include "mslam/slam/ICorrespondenceFinder.hh"


namespace mslam {

/// Nearest map point of every scan point, kept when it is closer than the
/// correspondence gate. The output is in scan order.
/// Sequential; the parallel registration path is the SYCL one
/// (DeviceCorrespondenceSearch).
class CorrespondenceFinder : public ICorrespondenceFinder {
public:
  void find(const IMap &map, const PointCloud &scan,
            float max_correspondence_distance,
            Correspondences &out) const override;
};

} // namespace mslam
