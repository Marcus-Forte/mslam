#include "slam/CorrespondenceFinder.hh"

#include "Timer.hh"

namespace mslam {
CorrespondenceFinder::CorrespondenceFinder(const std::shared_ptr<ILog> &logger)
    : logger_(logger) {}

Correspondences
CorrespondenceFinder::find(const IMap &map, const PointCloud &scan,
                           const PointCloud &transformed_scan,
                           float max_correspondence_distance) const {
  const auto num_points = std::min(scan.size(), transformed_scan.size());
  const float max_correspondence_distance_squared =
      max_correspondence_distance * max_correspondence_distance;

  Correspondences correspondences;
  correspondences.reserve(num_points);

  uint64_t knn_us = 0;
  Timer knn_timer;

  for (std::size_t index = 0; index < num_points; ++index) {
    const auto &scan_point = scan[index];
    const auto &transformed_scan_point = transformed_scan[index];
    const Point query(transformed_scan_point.x, transformed_scan_point.y,
                      transformed_scan_point.z);
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
                 correspondences.size(), num_points, knn_us);
  }

  return correspondences;
}

} // namespace mslam