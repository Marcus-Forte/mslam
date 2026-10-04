#include "slam/CorrespondenceFinder.hh"
#include <chrono>

namespace mslam {

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

  for (std::size_t index = 0; index < scan.size(); ++index) {
    const auto &scan_point = scan[index];
    const Point query(scan_point.x, scan_point.y, scan_point.z);
    const auto nearest = map.getClosestNeighbor(query);
    if (nearest.second >= max_correspondence_distance_squared) {
      continue;
    }

    correspondences.emplace_back(scan_point, nearest.first);
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