#pragma once

#include "ILog.hh"
#include "slam/ICorrespondenceFinder.hh"
#include <memory>

namespace mslam {

class CorrespondenceFinder : public ICorrespondenceFinder {
public:
  explicit CorrespondenceFinder(const std::shared_ptr<ILog> &logger);

  void find(const IMap &map, const PointCloud &scan,
            float max_correspondence_distance,
            Correspondences &out) const override;

private:
  std::shared_ptr<ILog> logger_;
};

} // namespace mslam