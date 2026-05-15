#include "slam/CorrespondenceFinder.hh"

#include "Timer.hh"

namespace mslam {
CorrespondenceFinder::CorrespondenceFinder(const std::shared_ptr<ILog> &logger)
    : logger_(logger) {}

Correspondences
CorrespondenceFinder::find(const IMap &map, const PointCloud &scan,
                           float max_correspondence_distance) const {

  const float max_correspondence_distance_squared =
      max_correspondence_distance * max_correspondence_distance;

  Correspondences correspondences;
  correspondences.reserve(scan.size());

  uint64_t knn_us = 0;
  Timer knn_timer;

  for (std::size_t index = 0; index < scan.size(); ++index) {
    const auto &scan_point = scan[index];
    const Point query(scan_point.x, scan_point.y, scan_point.z);
    knn_timer.start();
    const auto nearest = map.getClosestNeighbor(query);
    knn_us += knn_timer.stop();
    if (nearest.second >= max_correspondence_distance_squared) {
      continue;
    }

    correspondences.emplace_back(scan_point, nearest.first);
  }

  if (logger_ != nullptr) {
    logger_->log(ILog::Level::DEBUG,
                 "KNN Search. Correspondences: {} / {}. Took: {} us",
                 correspondences.size(), scan.size(), knn_us);
  }

  return correspondences;
}

} // namespace mslam