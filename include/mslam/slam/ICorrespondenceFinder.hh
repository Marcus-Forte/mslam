#pragma once

#include "mslam/common/Points.hh"
#include "mslam/map/IMap.hh"
#include <chrono>
#include <cstddef>
#include <memory>
#include <utility>

namespace mslam {

struct CorrespondenceSearchEvent {
  std::size_t scan_size = 0;
  std::size_t correspondence_count = 0;
  float max_correspondence_distance = 0;
  std::chrono::nanoseconds elapsed{};
};

class ICorrespondenceFinderObserver {
public:
  virtual ~ICorrespondenceFinderObserver() = default;

  virtual void
  onCorrespondenceSearch(const CorrespondenceSearchEvent & /*event*/) {}
};

class ICorrespondenceFinder {
public:
  virtual ~ICorrespondenceFinder() = default;

  void setObserver(std::shared_ptr<ICorrespondenceFinderObserver> observer) {
    observer_ = std::move(observer);
  }

  virtual void find(const IMap &map, const PointCloud &scan,
                    float max_correspondence_distance,
                    Correspondences &out) const = 0;

protected:
  std::shared_ptr<ICorrespondenceFinderObserver> observer_;
};

} // namespace mslam
