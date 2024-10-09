#include "Registration.hh"
#include "LevenbergMarquardt.hh"
#include "NumericalCost.hh"

namespace mslam {

constexpr int g_numIterations = 5;

Pose2D Registration::Align(const Pose2D &pose, IMap2D *map,
                           const PointCloud2D &scan) {
  LevenbergMarquardt lm;
}

} // namespace mslam