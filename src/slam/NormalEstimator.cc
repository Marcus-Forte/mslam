#include "slam/NormalEstimator.hh"
#include <Eigen/Eigenvalues>
#include <limits>

namespace mslam {

NormalEstimator::NormalEstimator(const IMap &map, int num_neighbors)
    : map_(map), num_neighbors_(num_neighbors) {}

std::size_t
NormalEstimator::MapPointKeyHash::operator()(const MapPointKey &key) const {
  const auto hash_x = std::hash<float>{}(key.x);
  const auto hash_y = std::hash<float>{}(key.y);
  const auto hash_z = std::hash<float>{}(key.z);
  return hash_x ^ (hash_y << 1) ^ (hash_z << 2);
}

std::optional<Eigen::Vector3d> NormalEstimator::estimate(const Point &query) {
  const MapPointKey key{query.x, query.y, query.z};
  auto [it, inserted] = cache_.try_emplace(key);
  if (inserted) {
    it->second =
        fromNeighbors(map_.getClosestNNeighbors(query, num_neighbors_));
  }
  return it->second;
}

std::optional<Eigen::Vector3d>
NormalEstimator::fromNeighbors(const std::vector<IMap::Neighbor> &neighbors) {
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

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      covariance, Eigen::ComputeEigenvectors);
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
