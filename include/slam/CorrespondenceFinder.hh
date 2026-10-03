#pragma once

#include "slam/ICorrespondenceFinder.hh"
#include <memory>
#include <spdlog/logger.h>

namespace mslam {

class CorrespondenceFinder : public ICorrespondenceFinder {
public:
  explicit CorrespondenceFinder(const std::shared_ptr<spdlog::logger> &logger);

  void find(const IMap &map, const PointCloud &scan,
            float max_correspondence_distance,
            Correspondences &out) const override;

private:
  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace mslam