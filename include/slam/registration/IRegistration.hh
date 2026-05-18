#pragma once

#include "ILog.hh"
#include "common/Points.hh"
#include "common/State.hh"
#include "map/IMap.hh"
#include "slam/ICorrespondenceFinder.hh"

namespace mslam {

class IRegistration {
public:
  IRegistration(int num_registration_iterations, int num_optimizer_iterations,
                float max_correspondence_distance,
                const std::shared_ptr<ILog> &logger,
                std::shared_ptr<ICorrespondenceFinder> correspondence_finder)
      : num_registration_iterations_(num_registration_iterations),
        num_optimizer_iterations_(num_optimizer_iterations),
        max_correspondence_distance_(max_correspondence_distance),
        logger_(logger),
        correspondence_finder_(std::move(correspondence_finder)) {}

  virtual ~IRegistration() = default;

  virtual SlamState Align(const SlamState &state, const IMap &map,
                          const PointCloud &scan) = 0;

protected:
  int num_registration_iterations_;
  int num_optimizer_iterations_;
  float max_correspondence_distance_;
  std::shared_ptr<ILog> logger_;
  std::shared_ptr<ICorrespondenceFinder> correspondence_finder_;
};

} // namespace mslam