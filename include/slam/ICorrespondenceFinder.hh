#pragma once

#include "common/Points.hh"
#include "map/IMap.hh"

namespace mslam {

class ICorrespondenceFinder {
public:
  virtual ~ICorrespondenceFinder() = default;

  virtual void find(const IMap &map, const PointCloud &scan,
                    float max_correspondence_distance,
                    Correspondences &out) const = 0;
};

} // namespace mslam
