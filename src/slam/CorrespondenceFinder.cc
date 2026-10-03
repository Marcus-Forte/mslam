#include "slam/CorrespondenceFinder.hh"

namespace mslam {
CorrespondenceFinder::CorrespondenceFinder(
    const std::shared_ptr<spdlog::logger> &logger)
    : logger_(logger) {}

void CorrespondenceFinder::find(const IMap &map, const PointCloud &scan,
                                float max_correspondence_distance,
                                Correspondences &correspondences) const {

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

  if (logger_ != nullptr) {
    logger_->debug("KNN Search. Correspondences: {} / {}",
                   correspondences.size(), scan.size());
  }
}

} // namespace mslam