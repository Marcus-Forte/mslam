#pragma once

#include "mslam/config/IConfig.hh"
#include "mslam/slam/ScanSequenceRegistration.hh"
#include "mslam/slam/posegraph/PoseGraph.hh"

#include <cstddef>
#include <memory>
#include <optional>
#include <spdlog/logger.h>
#include <vector>

namespace mslam {

/// Outcome of verifying one loop candidate.
struct LoopAttempt {
  std::size_t candidate = 0;
  bool accepted = false;
  std::size_t used_points = 0;
  std::size_t matched_points = 0;
  double matched_fraction = 0.0;
  double rms_m = 0.0;
};

struct LoopCloseResult {
  std::vector<LoopAttempt> attempts;
  std::vector<PoseEdge> edges; ///< One robust Loop edge per accepted attempt.
};

/// Finds and verifies loop closures for a keyframe whose mini map is sealed.
class LoopCloser {
public:
  /// At most this many candidates are verified per keyframe (the nearest ones
  /// in the map frame): each verification is a full ICP.
  static constexpr std::size_t kMaxCandidates = 1;

  /// Registration settings of a verification. The mini maps are already
  /// thinned at `mini_map_voxel_size`, the target map has voxels of 0.1 m, and
  /// the correspondence gate is 0.2 m: loose enough for the drift of a loop
  /// (a few cm to some 10 cm in the prior), tight enough that the two surfaces
  /// of a corner stay further apart than the gate (the
  /// "two walls meeting at a corner" pitfall of ScanSequenceRegistration drags
  /// them together within it). 20 rounds are an upper bound; ICP stops early
  /// when the pose settles.
  static ScanSequenceParameters
  verificationParameters(const PoseGraphParameters &parameters);

  LoopCloser(const PoseGraphParameters &parameters,
             std::shared_ptr<spdlog::logger> logger);

  /// Verifies loop candidates for keyframe @p new_id (the last of @p graph).
  /// Candidates are keyframes whose graph pose is within `loop_search_radius`
  /// of it and that are at least `loop_min_separation` keyframes older. The
  /// target is the candidate's mini map plus `loop_submap_keyframes`
  /// neighbours on each side (never newer than new_id - loop_min_separation),
  /// in the candidate's frame; the new mini map starts from the graph's
  /// relative pose. Accepted when matched/used >= loop_min_matched_fraction
  /// and rms <= loop_max_rms.
  LoopCloseResult close(const PoseGraph &graph, std::size_t new_id) const;

private:
  PoseGraphParameters parameters_;
  std::shared_ptr<spdlog::logger> logger_;
  /// Handed to the ScanSequenceRegistration of each verification, which logs
  /// the map search radius at info: this clone only shows warnings.
  std::shared_ptr<spdlog::logger> registration_logger_;
};

} // namespace mslam
