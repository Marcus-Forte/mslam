#include "mslam/slam/posegraph/LoopCloser.hh"

#include "mslam/slam/Transform.hh"

#include <algorithm>
#include <cmath>

namespace mslam {

namespace {

Pose3D toPose3D(const Eigen::Affine3d &pose) {
  const Eigen::Matrix3d rotation = pose.linear();
  const Eigen::Vector3d translation = pose.translation();
  const Eigen::Vector3d euler = rotationToEuler(rotation);
  Pose3D out;
  out << translation.x(), translation.y(), translation.z(), euler.x(),
      euler.y(), euler.z();
  return out;
}

} // namespace

ScanSequenceParameters
LoopCloser::verificationParameters(const PoseGraphParameters &parameters) {
  ScanSequenceParameters out;
  out.voxel_size = static_cast<float>(parameters.mini_map_voxel_size);
  out.map_resolution = 0.1F;
  out.max_points_per_voxel = 8;
  out.icp = IcpParameters{20, 5, 0.2F};
  out.num_threads = 1; // verification stays single-threaded
  return out;
}

LoopCloser::LoopCloser(const PoseGraphParameters &parameters,
                       std::shared_ptr<spdlog::logger> logger)
    : parameters_(parameters), logger_(std::move(logger)),
      registration_logger_(logger_->clone(logger_->name() + "-loop-icp")) {
  registration_logger_->set_level(
      std::max(logger_->level(), spdlog::level::warn)); // off stays off
}

LoopCloseResult LoopCloser::close(const PoseGraph &graph,
                                  std::size_t new_id) const {
  LoopCloseResult result;
  const std::size_t separation = parameters_.loop_min_separation;
  if (new_id >= graph.keyframes.size() || new_id < separation) {
    return result;
  }
  const Keyframe &fresh = graph.keyframes[new_id];
  const std::size_t newest_candidate = new_id - separation;

  // Nearest first; the id breaks ties so the choice is deterministic.
  std::vector<std::pair<double, std::size_t>> candidates;
  for (std::size_t id = 0; id <= newest_candidate; ++id) {
    const double distance =
        (graph.keyframes[id].pose.translation() - fresh.pose.translation())
            .norm();
    if (distance <= parameters_.loop_search_radius) {
      candidates.emplace_back(distance, id);
    }
  }
  std::sort(candidates.begin(), candidates.end());
  if (candidates.size() > kMaxCandidates) {
    candidates.resize(kMaxCandidates);
  }

  for (const auto &[distance, candidate] : candidates) {
    const Keyframe &target_kf = graph.keyframes[candidate];
    const Eigen::Affine3d target_from_world = inverseRigid(target_kf.pose);

    // Target: the candidate and its neighbours, in the candidate's frame.
    const std::size_t first =
        candidate > parameters_.loop_submap_keyframes
            ? candidate - parameters_.loop_submap_keyframes
            : 0;
    const std::size_t last = std::min<std::size_t>(
        candidate + parameters_.loop_submap_keyframes, newest_candidate);
    PointCloud target;
    for (std::size_t id = first; id <= last; ++id) {
      const Keyframe &kf = graph.keyframes[id];
      if (!kf.view) {
        continue;
      }
      PointCloud part = *kf.view;
      transformCloud(Eigen::Affine3d(target_from_world * kf.pose), part);
      target.insert(target.end(), part.begin(), part.end());
    }
    if (target.empty() || !fresh.view || fresh.view->empty()) {
      continue;
    }

    const Eigen::Affine3d prior = target_from_world * fresh.pose;
    ScanSequenceRegistration sequence(verificationParameters(parameters_),
                                      registration_logger_);
    sequence.add(target);
    const SequenceStep step =
        sequence.add(*fresh.view, std::optional<Pose3D>(toPose3D(prior)));

    LoopAttempt attempt;
    attempt.candidate = candidate;
    attempt.used_points = step.used_points;
    attempt.matched_points = step.matched_points;
    attempt.matched_fraction = step.used_points > 0
                                   ? static_cast<double>(step.matched_points) /
                                         static_cast<double>(step.used_points)
                                   : 0.0;
    attempt.rms_m = step.rms_m;
    attempt.accepted =
        step.registered && step.first_round.valid && step.matched_points > 0 &&
        attempt.matched_fraction >= parameters_.loop_min_matched_fraction &&
        attempt.rms_m <= parameters_.loop_max_rms;

    logger_->debug("loop {} -> {}: {} (matched {:.1f} % of {} points, rms "
                   "{:.1f} mm)",
                   candidate, new_id,
                   attempt.accepted ? "accepted" : "rejected",
                   100.0 * attempt.matched_fraction, attempt.used_points,
                   1000.0 * attempt.rms_m);
    result.attempts.push_back(attempt);

    if (attempt.accepted) {
      PoseEdge edge;
      edge.kind = EdgeKind::Loop;
      edge.from = candidate;
      edge.to = new_id;
      edge.measured = toAffine(step.pose);
      edge.information = informationFromSigmas(
          parameters_.loop_sigma_translation, parameters_.loop_sigma_rotation);
      edge.robust = true;
      edge.verify_rms = attempt.rms_m;
      edge.verify_matched_fraction = attempt.matched_fraction;
      result.edges.push_back(edge);
    }
  }
  return result;
}

} // namespace mslam
