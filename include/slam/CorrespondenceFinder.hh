#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "map/IMap.hh"
#include <memory>

namespace mslam {

class CorrespondenceFinder {
public:
  explicit CorrespondenceFinder(const std::shared_ptr<ILog> &logger);

  Correspondences find(const IMap &map, const PointCloud &scan,
                       const PointCloud &transformed_scan,
                       float max_correspondence_distance) const;

private:
  std::shared_ptr<ILog> logger_;
};

} // namespace mslam