#include "mslam/slam/CorrespondenceFinder.hh"
#include <chrono>
#include <utility>

namespace mslam {

namespace {

// Minimum scan points per task; scans shorter than twice this run inline. A
// live scan has about 1000 points after filtering. 64, 128 and 256 measured
// the same on living_room_garden at 4 threads (sweep in improvement_tbb.md).
constexpr std::size_t kSearchGrain = 128;

} // namespace

CorrespondenceFinder::CorrespondenceFinder(
    std::shared_ptr<const ParallelExecutor> executor)
    : executor_(std::move(executor)) {}

void CorrespondenceFinder::find(const IMap &map, const PointCloud &scan,
                                float max_correspondence_distance,
                                Correspondences &correspondences) const {
  const auto start = observer_ != nullptr
                         ? std::chrono::steady_clock::now()
                         : std::chrono::steady_clock::time_point{};

  const float max_correspondence_distance_squared =
      max_correspondence_distance * max_correspondence_distance;

  correspondences.clear();
  correspondences.reserve(scan.size());

  if (executor_ == nullptr) {
    for (std::size_t index = 0; index < scan.size(); ++index) {
      const auto &scan_point = scan[index];
      const Point query(scan_point.x, scan_point.y, scan_point.z);
      const auto nearest = map.getClosestNeighbor(query);
      if (nearest.second >= max_correspondence_distance_squared) {
        continue;
      }

      correspondences.emplace_back(scan_point, nearest.first);
    }
  } else {
    // Same search per index as above, each writing only its own slot; the
    // gate and the compaction then run in scan order, so the output does not
    // depend on the thread count.
    nearest_.resize(scan.size());
    executor_->forRange(
        scan.size(), kSearchGrain, [&](std::size_t begin, std::size_t end) {
          for (std::size_t index = begin; index < end; ++index) {
            const auto &scan_point = scan[index];
            nearest_[index] = map.getClosestNeighbor(
                Point(scan_point.x, scan_point.y, scan_point.z));
          }
        });
    for (std::size_t index = 0; index < scan.size(); ++index) {
      if (nearest_[index].second >= max_correspondence_distance_squared) {
        continue;
      }
      correspondences.emplace_back(scan[index], nearest_[index].first);
    }
  }

  if (observer_ != nullptr) {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    observer_->onCorrespondenceSearch(
        {.scan_size = scan.size(),
         .correspondence_count = correspondences.size(),
         .max_correspondence_distance = max_correspondence_distance,
         .elapsed =
             std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)});
  }
}

} // namespace mslam
