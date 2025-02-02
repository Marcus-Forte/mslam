#pragma once

#include <ostream>

namespace mslam {

enum class MapType { KdTree, Voxel };

/**
 * @brief Algorithm Paramters.
 *
 */
struct SlamParameters {

  int opt_iterations = 5;
};

/**
 * @brief SLAM Configuraton.
 *
 */
struct SlamConfiguration {
  bool with_imu = false;
  bool with_lidar = true;
  MapType map_type = MapType::KdTree;
  std::string remote_scanner;

  SlamParameters parameters;

  friend std::ostream &operator<<(std::ostream &os, SlamConfiguration config) {
    os << "IMU: " << config.with_imu << "\n"
       << "LIDAR: " << config.with_lidar << "\n"
       << "Map Type: "
       << (config.map_type == MapType::KdTree ? "KDtree" : "Voxel") << "\n"
       << "Scanner: " << config.remote_scanner << "\n"
       << "# SLAM Parameters #" << "\n\n"
       << "Optimizer Iterations: " << config.parameters.opt_iterations;
    return os;
  }
};

class IConfig {
public:
  virtual void load() = 0;
  inline SlamConfiguration getConfig() const { return config_; }

protected:
  SlamConfiguration config_;
};

} // namespace mslam