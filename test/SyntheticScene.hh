#pragma once

// A small synthetic room (a floor and two walls) sampled at random points,
// shared by the registration and pipeline tests.

#include "mslam/common/Points.hh"

#include <random>

namespace test {

/// Random points on a floor (z = -1) and on the walls x = 3 and y = 3, about
/// @p density points per square metre on each surface. Surfaces span
/// [-extent, extent] along their tangents (walls: z in [-1.5, 1.5]). Sample the
/// map with the full extent and a scan with a smaller one, so that a shifted
/// scan stays inside the map. Different seeds give different samplings of the
/// same surfaces, as consecutive scans would.
///
/// Random rather than gridded samples: a regular grid thinned to a few points
/// per voxel leaves nearly collinear neighbourhoods, whose normals are
/// meaningless.
inline mslam::PointCloud sampleRoom(float density = 400.0F, unsigned seed = 1,
                                    float extent = 3.0F) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> along(-extent, extent);
  std::uniform_real_distribution<float> height(-1.5F, 1.5F);
  const auto per_surface =
      static_cast<int>(density * (2.0F * extent) * (2.0F * extent));

  mslam::PointCloud cloud;
  cloud.reserve(static_cast<std::size_t>(per_surface) * 3);
  for (int i = 0; i < per_surface; ++i) {
    cloud.emplace_back(along(rng), along(rng), -1.0F, 100.0F); // floor
    cloud.emplace_back(3.0F, along(rng), height(rng), 100.0F); // wall x = 3
    cloud.emplace_back(along(rng), 3.0F, height(rng), 100.0F); // wall y = 3
  }
  return cloud;
}

} // namespace test
