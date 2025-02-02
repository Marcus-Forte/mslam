#include "ConsoleLogger.hh"
#include "LevenbergMarquardt.hh"
#include "NumericalCost.hh"
#include "Registration.hh"
#include "Transform.hh"
#include "common/Points.hh"
#include <memory>

const float g_maxDistance = 0.1; //
const int g_numRegIterations = 10;
namespace mslam {

struct Model {
  Model(const Eigen::VectorXd &x) : transform_{toAffine(x[0], x[1], x[2])} {}

  // Error function
  Eigen::Vector2d operator()(const Eigen::Vector2d &input,
                             const Eigen::Vector2d &observation) {
    return input - transform_ * observation;
  }

private:
  Eigen::Affine2d transform_;
};

constexpr int g_numIterations = 5;

Pose2D Registration::Align(const Pose2D &pose, const IMap &map,
                           const PointCloud3 &scan) {

  PointCloud3 src;
  PointCloud3 tgt;

  // Initial guess.
  Eigen::VectorXd x{{pose[0], pose[1], pose[2]}};
  Eigen::Affine2d transform_;

  for (int i = 0; i < g_numRegIterations; ++i) {
    transform_ = toAffine(x[0], x[1], x[2]);
    // Data association
    src.clear();
    tgt.clear();
    src.reserve(scan.size());
    tgt.reserve(scan.size());
    for (const auto &pt : scan) {
      const Eigen::Vector2d point_eigen{pt.x, pt.y};
      const auto transformed_scan_pt = transform_ * point_eigen;
      /// \todo optimize
      const Point3 query(transformed_scan_pt.x(), transformed_scan_pt.y(), 0);
      auto closest = map.getClosestNeighbor(query);
      if (closest.second < g_maxDistance * g_maxDistance) {
        // source, target
        src.emplace_back(pt);
        tgt.emplace_back(closest.first); // map
      }
    }
    // Align
    auto logger = std::make_shared<ConsoleLogger>();
    logger->log(ILog::Level::INFO, "Cors: {}", tgt.size());
    logger->setLevel(ILog::Level::INFO);
    LevenbergMarquardt lm(logger);
    lm.setMaxIterations(10);
    auto cost = std::make_shared<
        NumericalCost<Eigen::Vector2d, Eigen::Vector2d, Model>>(&tgt, &src);

    lm.addCost(cost);
    lm.optimize(x);
  }

  return {x[0], x[1], x[2]};
}

} // namespace mslam