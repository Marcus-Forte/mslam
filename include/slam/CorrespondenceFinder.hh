#pragma once

#include "slam/ICorrespondenceFinder.hh"

namespace mslam {

class CorrespondenceFinder : public ICorrespondenceFinder {
public:
  CorrespondenceFinder() = default;

  void find(const IMap &map, const PointCloud &scan,
            float max_correspondence_distance,
            Correspondences &out) const override;
};

} // namespace mslam