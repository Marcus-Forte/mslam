#include "Registration.hh"
#include "ConsoleLogger.hh"
#include "LevenbergMarquardt.hh"
#include "NumericalCost.hh"
#include "Transform.hh"
#include "common/Points.hh"
#include <memory>

const float g_maxDistance = 0.05; //
const int g_numRegIterations = 10;
namespace mslam {

struct Model {
  Model(const Eigen::VectorXd &x) : transform_{toAffine(x[0], x[1], x[2])} {}
  mslam::Point2 operator()(const mslam::Point2 &input,
                           const mslam::Point2 &observation) {
    return input - transform_ * observation;
  }

private:
  Eigen::Affine2d transform_;
};

constexpr int g_numIterations = 5;

Pose2D Registration::Align(const Pose2D &pose, const IMap2D &map,
                           const PointCloud2 &scan) {

  std::vector<mslam::Point2> src;
  std::vector<mslam::Point2> tgt;

  Eigen::VectorXd x{{pose[0], pose[1], pose[2]}};

  Eigen::Affine2d transform_;

  for (int i = 0; i < g_numRegIterations; ++i) {
    Eigen::Affine2d transform_ = toAffine(x[0], x[1], x[2]);
    // Data association
    src.clear();
    tgt.clear();
    src.reserve(scan.size());
    tgt.reserve(scan.size());
    for (const auto &pt : scan) {
      const auto transformed_scan_pt = transform_ * pt;
      auto closest = map.getClosestNeighbor(transformed_scan_pt);
      if (closest.second < g_maxDistance * g_maxDistance) {
        // source, target
        src.emplace_back(pt);
        tgt.emplace_back(closest.first); // map
      }
    }
    // Align
    auto logger = std::make_shared<ConsoleLogger>();
    logger->setLevel(ILog::Level::ERROR);
    LevenbergMarquardt lm(logger);
    lm.setMaxIterations(10);
    auto cost =
        std::make_shared<NumericalCost<mslam::Point2, mslam::Point2, Model>>(
            &tgt, &src);

    lm.addCost(cost);
    lm.optimize(x);

    transform_ = toAffine(x[0], x[1], x[2]);
  }

  // Update transformed source

  return {x[0], x[1], x[2]};
}

} // namespace mslam