#pragma once

#include "mslam/OptimizerObserver.hh"
#include "mslam/common/Points.hh"
#include "mslam/map/IMap.hh"
#include "mslam/slam/ICorrespondenceFinder.hh"
#include "mslam/slam/NormalEstimator.hh"
#include "mslam/utils/Parallel.hh"

#include "moptim/LevenbergMarquardt.hh"

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <spdlog/logger.h>
#include <vector>

namespace mslam {

/// Settings shared by the ICP-style registrations.
struct IcpParameters {
  int registration_iterations = 5; ///< Correspondence re-association rounds.
  int optimizer_iterations = 5;    ///< Optimizer iterations per round.
  float max_correspondence_distance = 0.1F;
};

/// Size of the pose change the first registration round of an Align() applied:
/// the norms of the translation and rotation parts of
/// se3Log(T_after * T_before^-1). It measures how wrong the prior was. `valid`
/// is false if that round found no correspondences.
struct FirstRoundIncrement {
  bool valid = false;
  double translation_m = 0.0;
  double rotation_rad = 0.0;
};

/**
 * @brief State and helpers shared by the registrations: configuration, the
 * moving source cloud, the correspondence search and the point-to-plane
 * constraint buffers.
 *
 * It is a concrete helper that a registration owns, not a base class, so each
 * registration exposes only the `Align` it actually supports. Buffers keep
 * their capacity between `Align` calls.
 */
class IcpWorkspace {
public:
  /// @param executor Runs the normal estimation of new map points (nullptr:
  /// sequential). The correspondence search runs on whatever executor
  /// @p correspondence_finder was given.
  IcpWorkspace(const IcpParameters &parameters,
               std::shared_ptr<spdlog::logger> logger,
               std::shared_ptr<ICorrespondenceFinder> correspondence_finder,
               std::shared_ptr<const ParallelExecutor> executor = nullptr);

  const IcpParameters &parameters() const { return parameters_; }
  spdlog::logger &logger() const { return *logger_; }

  /// Starts a registration of @p scan against @p map: the source cloud
  /// becomes @p scan transformed by @p initial_guess. @p scan must outlive the
  /// registration (it is read again by resetSource()).
  void begin(const IMap &map, const PointCloud &scan,
             const Eigen::Affine3d &initial_guess);

  /// Moves the source cloud by @p delta (applied on the left).
  void moveSource(const Eigen::Affine3d &delta);

  /// Rebuilds the source cloud as the scan transformed by @p total.
  void resetSource(const Eigen::Affine3d &total);

  const PointCloud &source() const { return source_; }

  /// Finds the correspondences of the source cloud in the map. Their count
  /// is correspondences().size().
  void findCorrespondences(const IMap &map);
  const Correspondences &correspondences() const { return correspondences_; }

  /// findCorrespondences() plus a surface normal per matched map point, from
  /// which the point-to-plane constraints are assembled (points without a
  /// normal are dropped). Returns the number of constraints.
  ///
  /// Constraint i is the pair (planeInputs()[i], planeOffsets()[i]) =
  /// ([source_xyz, normal_xyz], normal . map_point), the layout of
  /// Point3PlaneDistance.
  std::size_t findPlaneConstraints(const IMap &map);

  std::span<const Eigen::Matrix<double, 6, 1>> planeInputs() const {
    return plane_inputs_;
  }
  std::span<const double> planeOffsets() const { return plane_offsets_; }

  /// Forwards the optimizer telemetry to the logger, but only when debug
  /// logging is enabled: with no observer attached the optimizer neither reads
  /// the clock nor builds event payloads.
  void attachObserver(moptim::LevenbergMarquardt<double> &optimizer);

private:
  IcpParameters parameters_;
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<ICorrespondenceFinder> correspondence_finder_;
  std::shared_ptr<const ParallelExecutor> executor_;
  NormalEstimator normal_estimator_;
  std::vector<std::uint32_t> normal_slots_; ///< One per correspondence.
  std::optional<OptimizerObserver<double>> observer_;

  const PointCloud *scan_ = nullptr;
  PointCloud source_;
  Correspondences correspondences_;
  std::vector<Eigen::Matrix<double, 6, 1>> plane_inputs_;
  std::vector<double> plane_offsets_;
};

} // namespace mslam
