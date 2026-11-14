#include "mslam/slam/NormalEstimator.hh"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <bit>
#include <limits>

namespace mslam {

NormalEstimator::NormalEstimator(int num_neighbors)
    : num_neighbors_(std::clamp(num_neighbors, 0, kMaxNeighbors)) {}

void NormalEstimator::reset(const IMap &map) {
  map_ = &map;
  cache_.clear();
}

std::optional<Eigen::Vector3d> NormalEstimator::estimate(const Point &query) {
  const MapPointKey key{std::bit_cast<std::uint32_t>(query.x),
                        std::bit_cast<std::uint32_t>(query.y),
                        std::bit_cast<std::uint32_t>(query.z)};
  auto [it, inserted] = cache_.try_emplace(key);
  if (inserted) {
    std::array<IMap::Neighbor, kMaxNeighbors> neighbors;
    const auto count = map_->findClosestNeighbors(
        query,
        std::span(neighbors.data(), static_cast<std::size_t>(num_neighbors_)));
    it.value() = fromNeighbors(std::span(neighbors.data(), count));
  }
  return it->second;
}

std::optional<Eigen::Vector3d>
NormalEstimator::fromNeighbors(std::span<const IMap::Neighbor> neighbors) {
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
    const Eigen::Vector3d centered =
        Eigen::Vector3d{point.x, point.y, point.z} - centroid;
    covariance.noalias() += centered * centered.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(covariance, Eigen::ComputeEigenvectors);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }

  const Eigen::Vector3d normal = solver.eigenvectors().col(0);
  const double norm = normal.norm();
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  return normal / norm;
}

} // namespace mslam
