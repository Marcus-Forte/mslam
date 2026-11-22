#include "mslam/slam/NormalEstimator.hh"
#include "mslam/slam/NormalMath.hh"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <bit>
#include <limits>

namespace mslam {

NormalEstimator::NormalEstimator(int num_neighbors)
    : num_neighbors_(std::clamp(num_neighbors, 0, kMaxNeighbors)) {}

namespace {

// Minimum new map points per task in estimateAll() (each costs a 5-NN search
// and a 3x3 eigen-solve). Lists shorter than twice this run inline. 16 beat 32
// and 64 on living_room_garden at 4 threads, because the short lists of the
// later registration rounds then still run in parallel; 8 was no better (sweep
// in improvement_tbb.md).
constexpr std::size_t kNormalGrain = 16;

} // namespace

void NormalEstimator::reset(const IMap &map) {
  map_ = &map;
  cache_.clear();
  normals_.clear();
}

NormalEstimator::MapPointKey NormalEstimator::keyOf(const Point &point) {
  return {std::bit_cast<std::uint32_t>(point.x),
          std::bit_cast<std::uint32_t>(point.y),
          std::bit_cast<std::uint32_t>(point.z)};
}

std::optional<Eigen::Vector3d>
NormalEstimator::compute(const Point &query) const {
  std::array<IMap::Neighbor, kMaxNeighbors> neighbors;
  const auto count = map_->findClosestNeighbors(
      query,
      std::span(neighbors.data(), static_cast<std::size_t>(num_neighbors_)));
  return fromNeighbors(std::span(neighbors.data(), count));
}

std::optional<Eigen::Vector3d> NormalEstimator::estimate(const Point &query) {
  const auto slot = static_cast<std::uint32_t>(normals_.size());
  auto [it, inserted] = cache_.try_emplace(keyOf(query), slot);
  if (inserted) {
    normals_.push_back(compute(query));
  }
  return normals_[it->second];
}

void NormalEstimator::estimateAll(const Correspondences &correspondences,
                                  std::vector<std::uint32_t> &slots,
                                  const ParallelExecutor *executor) {
  // Phase 1, sequential: resolve every slot and allocate the new ones, so
  // normals_ is not resized while phase 2 writes into it.
  slots.resize(correspondences.size());
  pending_.clear();
  for (std::size_t i = 0; i < correspondences.size(); ++i) {
    const Point &map_point = correspondences[i].second;
    const auto slot = static_cast<std::uint32_t>(normals_.size());
    auto [it, inserted] = cache_.try_emplace(keyOf(map_point), slot);
    if (inserted) {
      normals_.emplace_back();
      pending_.emplace_back(map_point, slot);
    }
    slots[i] = it->second;
  }

  // Phase 2: each new slot is written by exactly one index.
  const auto fill = [this](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      const auto &[point, slot] = pending_[i];
      normals_[slot] = compute(point);
    }
  };
  if (executor == nullptr) {
    fill(0, pending_.size());
  } else {
    executor->forRange(pending_.size(), kNormalGrain, fill);
  }
}

std::optional<Eigen::Vector3d>
NormalEstimator::fromNeighbors(std::span<const IMap::Neighbor> neighbors) {
  Eigen::Vector3d normal;
  if (!normalFromPoints(
          neighbors.size(),
          [&](std::size_t i) -> const Point & { return neighbors[i].first; },
          normal)) {
    return std::nullopt;
  }
  return normal;
}

} // namespace mslam
