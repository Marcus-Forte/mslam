#pragma once

#include "mslam/slam/ICorrespondenceFinder.hh"
#include "mslam/utils/Parallel.hh"

#include <memory>
#include <vector>

namespace mslam {

/// Nearest map point of every scan point, kept when it is closer than the
/// correspondence gate. The output is in scan order.
///
/// With an executor the searches run in parallel and give the same output,
/// element for element, as without one. They then go through a scratch buffer
/// owned by the finder, so one finder serves one find() at a time.
class CorrespondenceFinder : public ICorrespondenceFinder {
public:
  explicit CorrespondenceFinder(
      std::shared_ptr<const ParallelExecutor> executor = nullptr);

  void find(const IMap &map, const PointCloud &scan,
            float max_correspondence_distance,
            Correspondences &out) const override;

private:
  std::shared_ptr<const ParallelExecutor> executor_;
  mutable std::vector<IMap::Neighbor> nearest_; ///< Parallel path scratch.
};

} // namespace mslam
