#include "slam/Registration.hh"
#include "Timer.hh"
#include "common/Points.hh"
#include "moptim/LevenbergMarquardt.h"
#include "moptim/NumericalCostForwardEuler.h"
#include "slam/CorrespondenceFinder.hh"
#include "slam/Transform.hh"
#include <Eigen/Eigenvalues>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>

#include "PointDistance.hh"

namespace mslam {

constexpr int g_maxSmallDeltaHits = 3;
constexpr int g_normalEstimationNeighbors = 5;

namespace {

struct MapPointKey {
  float x;
  float y;
  float z;

  bool operator==(const MapPointKey &) const = default;
};

struct MapPointKeyHash {
  std::size_t operator()(const MapPointKey &key) const {
    const auto hash_x = std::hash<float>{}(key.x);
    const auto hash_y = std::hash<float>{}(key.y);
    const auto hash_z = std::hash<float>{}(key.z);

    return hash_x ^ (hash_y << 1) ^ (hash_z << 2);
  }
};

inline MapPointKey makeMapPointKey(const Point &point) {
  return {point.x, point.y, point.z};
}

std::optional<Eigen::Vector3d>
estimateSurfaceNormal(const std::vector<IMap::Neighbor> &neighbors) {
  if (neighbors.size() < 3) {
    return std::nullopt;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto &[point, _] : neighbors) {
    centroid += Eigen::Vector3d{point.x, point.y, point.z};
  }
  centroid /= static_cast<double>(neighbors.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &[point, _] : neighbors) {
    const Eigen::Vector3d centered_point =
        Eigen::Vector3d{point.x, point.y, point.z} - centroid;
    covariance.noalias() += centered_point * centered_point.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      covariance, Eigen::ComputeEigenvectors);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }

  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  const double norm = normal.norm();
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  return normal / norm;
}

std::optional<Eigen::Vector3d>
estimateSurfaceNormal(const IMap &map, const Point &query, int num_neighbors) {
  const auto neighbors = map.getClosestNNeighbors(query, num_neighbors);
  return estimateSurfaceNormal(neighbors);
}

void updateMapCorrespondencePoints(const Correspondences &correspondences,
                                   VectorPoint3d &map_points) {
  map_points.clear();
  map_points.reserve(correspondences.size());

  for (const auto &correspondence : correspondences) {
    const auto &map_point = correspondence.second;
    map_points.emplace_back(map_point.x, map_point.y, map_point.z);
  }
}

template <int InputDim, bool UseNormals>
uint64_t buildMetricInputs(
    const IMap &map, const Correspondences &correspondences,
    std::vector<Eigen::Matrix<double, InputDim, 1>> &metric_inputs,
    VectorPoint3d &map_points) {
  metric_inputs.clear();
  metric_inputs.reserve(correspondences.size());
  map_points.clear();
  map_points.reserve(correspondences.size());

  uint64_t normal_estimation_us = 0;
  [[maybe_unused]] std::unordered_map<MapPointKey,
                                      std::optional<Eigen::Vector3d>,
                                      MapPointKeyHash> normal_cache;
  if constexpr (UseNormals) {
    normal_cache.reserve(correspondences.size());
  }

  for (const auto &[scan_point, map_point] : correspondences) {
    Eigen::Matrix<double, InputDim, 1> metric_input;
    metric_input.template head<3>() << scan_point.x, scan_point.y, scan_point.z;

    if constexpr (UseNormals) {
      const auto cache_key = makeMapPointKey(map_point);
      auto [cache_it, inserted] = normal_cache.try_emplace(cache_key);
      if (inserted) {
        Timer normal_timer;
        normal_timer.start();
        cache_it->second =
            estimateSurfaceNormal(map, map_point, g_normalEstimationNeighbors);
        normal_estimation_us += normal_timer.stop();
      }
      if (!cache_it->second.has_value()) {
        continue;
      }
      metric_input.template tail<3>() = *cache_it->second;
    }

    metric_inputs.push_back(metric_input);
    map_points.emplace_back(map_point.x, map_point.y, map_point.z);
  }

  return normal_estimation_us;
}

template <typename Model, int InputDim, bool UseNormals>
Pose3D
align3DWithMetric(const Pose3D &pose, const IMap &map, const PointCloud &scan,
                  VectorPoint3d &last_map_correspondences,
                  int num_registration_iterations, int num_optimizer_iterations,
                  float max_correspondence_distance,
                  const std::shared_ptr<ILog> &logger) {
  std::vector<Eigen::Matrix<double, InputDim, 1>> metric_inputs;

  Eigen::VectorXd x{{pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]}};
  Eigen::Affine3d transform;
  CorrespondenceFinder correspondence_finder(logger);

  int small_delta_hits = 0;
  Timer iteration_timer;
  Timer stage_timer;

  PointCloud transformed_scan;
  transformed_scan.resize(scan.size());

  for (int i = 0; i < num_registration_iterations; ++i) {
    iteration_timer.start();
    transform = toAffine(x[0], x[1], x[2], x[3], x[4], x[5]);

    for (std::size_t index = 0; index < scan.size(); ++index) {
      const auto &pt = scan[index];
      const Eigen::Vector3d point_eigen{pt.x, pt.y, pt.z};
      const Eigen::Vector3d transformed_point = transform * point_eigen;
      transformed_scan[index].x = transformed_point.x();
      transformed_scan[index].y = transformed_point.y();
      transformed_scan[index].z = transformed_point.z();
    }
    // transformCloud(transform, transformed_scan);

    stage_timer.start();
    const auto correspondences = correspondence_finder.find(
        map, scan, transformed_scan, max_correspondence_distance);
    const auto normal_estimation_us = buildMetricInputs<InputDim, UseNormals>(
        map, correspondences, metric_inputs, last_map_correspondences);

    const auto correspondence_us = stage_timer.stop();

    if (last_map_correspondences.empty()) {
      logger->log(
          ILog::Level::WARNING,
          "3D registration found no correspondences; returning prior pose.");
      break;
    }

    logger->log(ILog::Level::DEBUG,
                "Correspondence Search. Correspondences: {} / {}. Normal Est.: "
                "{} us. Took: {} us",
                last_map_correspondences.size(), scan.size(),
                normal_estimation_us, correspondence_us);

    stage_timer.start();
    moptim::LevenbergMarquardt<double> lm(6, logger);
    lm.setMaxIterations(num_optimizer_iterations);

    auto cost =
        std::make_shared<moptim::NumericalCostForwardEuler<Model, double>>(
            metric_inputs[0].data(), last_map_correspondences[0].data(),
            last_map_correspondences.size(), InputDim, 3, 6);

    lm.addCost(cost);
    const auto status = lm.optimize(x.data());

    const auto optimization_us = stage_timer.stop();
    logger->log(ILog::Level::DEBUG, "Opt. Took: {} us", optimization_us);
    logger->log(ILog::Level::DEBUG,
                "Registration Iteration: {}/{}. Total: {} us", i + 1,
                num_registration_iterations, iteration_timer.stop());

    if (status == moptim::Status::SMALL_DELTA) {
      small_delta_hits++;
      if (small_delta_hits > g_maxSmallDeltaHits) {
        break;
      }
    }
  }

  return {x[0], x[1], x[2], x[3], x[4], x[5]};
}

} // namespace

Registration::Registration(int num_registration_iterations,
                           int num_optimizer_iterations,
                           float max_correspondence_distance,
                           const std::shared_ptr<ILog> &logger)
    : num_registration_iterations_(num_registration_iterations),
      num_optimizer_iterations_(num_optimizer_iterations),
      max_correspondence_distance_(max_correspondence_distance),
      logger_(logger) {}

/// \todo add transformed scan as output
Pose2D Registration::Align2D(const Pose2D &pose, const IMap &map,
                             const PointCloud &scan) {

  VectorPoint2d scan_points;
  VectorPoint2d map_correspondences;

  // Initial guess
  Eigen::VectorXd x{{pose[0], pose[1], pose[2]}};
  Eigen::Affine2d transform_;

  static Timer timer;
  int small_delta_hits = 0;

  for (int i = 0; i < num_registration_iterations_; ++i) {
    timer.start();
    transform_ = toAffine(x[0], x[1], x[2]);
    // Data association

    scan_points.clear();
    map_correspondences.clear();
    scan_points.reserve(scan.size());
    map_correspondences.reserve(scan.size());

    // Find point correspondences
    for (const auto &pt : scan) {
      const Eigen::Vector2d point_eigen{pt.x, pt.y};
      const Eigen::Vector2d transformed_scan_pt = transform_ * point_eigen;

      /// \todo optimize
      const Point query(transformed_scan_pt.x(), transformed_scan_pt.y(), 0);
      const auto closest = map.getClosestNeighbor(query);
      if (closest.second <
          max_correspondence_distance_ * max_correspondence_distance_) {

        scan_points.emplace_back(pt.x, pt.y); // untransformed observation
        map_correspondences.emplace_back(closest.first.x,
                                         closest.first.y); // point in map
      }
    }

    // Align
    moptim::LevenbergMarquardt<double> lm(3, logger_);

    lm.setMaxIterations(num_optimizer_iterations_);

    auto cost = std::make_shared<
        moptim::NumericalCostForwardEuler<Point2Distance, double>>(
        map_correspondences[0].data(), scan_points[0].data(),
        map_correspondences.size(), 2, 2, 3);

    lm.addCost(cost);
    const auto status = lm.optimize(x.data());

    const auto delta = timer.stop();
    logger_->log(
        ILog::Level::DEBUG,
        "Reg. Iteration: {}/ {}. Correspondences: {} / {}. Took: {} us", i + 1,
        num_registration_iterations_, map_correspondences.size(), scan.size(),
        delta);

    if (registration_callback2d_) {
      registration_callback2d_({x[0], x[1], x[2]}, map_correspondences,
                               scan_points);
    }

    if (status == moptim::Status::SMALL_DELTA) {
      small_delta_hits++;

      if (small_delta_hits > g_maxSmallDeltaHits) {
        break;
      }
    }
  }

  return {x[0], x[1], x[2]};
}

Pose3D Registration::Align3D(const Pose3D &pose, const IMap &map,
                             const PointCloud &scan,
                             RegistrationMetric3D metric) {
  switch (metric) {
  case RegistrationMetric3D::PointToPoint:
    return align3DWithMetric<Point3Distance, 3, false>(
        pose, map, scan, last_map_correspondences_,
        num_registration_iterations_, num_optimizer_iterations_,
        max_correspondence_distance_, logger_);
  case RegistrationMetric3D::PointToPlane:
    return align3DWithMetric<Point3PlaneDistance, 6, true>(
        pose, map, scan, last_map_correspondences_,
        num_registration_iterations_, num_optimizer_iterations_,
        max_correspondence_distance_, logger_);
  }

  throw std::runtime_error("Unsupported 3D registration metric");
}

void Registration::registerIterationCallback2D(
    RegistrationCallback2D &&callback) {
  registration_callback2d_ = callback;
}
} // namespace mslam