#pragma once

#include "mslam/OptimizerObserver.hh"
#include "mslam/common/Points.hh"
#include "mslam/map/IMap.hh"
#include "mslam/slam/ICorrespondenceFinder.hh"
#include "mslam/slam/NormalEstimator.hh"
#include "mslam/slam/registration/DeviceCorrespondenceSearch.hh"
#include "mslam/utils/Parallel.hh"

#include "moptim/LevenbergMarquardt.hh"

#include <Eigen/Dense>
#include <chrono>
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

/// Wall time spent in the parts of the registrations, summed over every Align
/// (bench_slam prints it). search: the 1-NN correspondence search; normals: the
/// normal estimation of the matched map points; assembly: building the
/// point-to-plane constraints; solve: building the costs and the optimizer
/// runs. Reading the clock a few times per round costs nothing measurable. On a
/// map with a device mirror (SyclVoxelHashMap) the search and the normals are
/// one kernel, counted as search, and normals stays zero.
struct RegistrationTimings {
  std::chrono::nanoseconds search{0};
  std::chrono::nanoseconds normals{0};
  std::chrono::nanoseconds assembly{0};
  std::chrono::nanoseconds solve{0};
  /// Device path: matched map points whose normal was computed / read from the
  /// map's cache.
  std::size_t normals_computed = 0;
  std::size_t normals_cached = 0;

  RegistrationTimings &operator+=(const RegistrationTimings &other) {
    search += other.search;
    normals += other.normals;
    assembly += other.assembly;
    solve += other.solve;
    normals_computed += other.normals_computed;
    normals_cached += other.normals_cached;
    return *this;
  }
};

/// Whether a registration round moved the pose from @p before to @p after by
/// less than 1 micrometre and 1 microradian (se3Log of the relative pose):
/// rounds that no longer move it are what ends an ICP loop early.
///
/// The registrations used to count rounds whose optimizer returned SMALL_DELTA
/// instead. That status only says the round's own solve converged (its last
/// tiny step was rejected on a rounding-level cost increase), not that the
/// re-associated correspondences stopped moving the pose: point-to-point ICP
/// on planar scenes converges every round while still sliding millimetres per
/// round, and the old criterion stopped it after five rounds.
bool isNegligibleRoundStep(const Eigen::Affine3d &before,
                           const Eigen::Affine3d &after);

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
  /// On a map with a device mirror (IMap::deviceMap()) the search and the
  /// normals run in one SYCL kernel (DeviceCorrespondenceSearch) on as many
  /// work-groups as the executor has threads, with the normals cached in the
  /// map across scans; the constraints are assembled in the same order and with
  /// the same arithmetic as on the host path.
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

  /// Time spent so far (findPlaneConstraints() fills search, normals and
  /// assembly; the registrations add their solve time).
  RegistrationTimings &timings() { return timings_; }
  const RegistrationTimings &timings() const { return timings_; }

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
  RegistrationTimings timings_;
  DeviceCorrespondenceSearch device_search_;
};

} // namespace mslam
