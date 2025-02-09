#pragma once

#include <ostream>

namespace mslam {

enum class MapType { KdTree, Voxel };

/**
 * @brief Algorithm Paramters.
 *
 */
struct SlamParameters {
  unsigned int opt_iterations = 5;
  unsigned int reg_iterations = 5;
  float max_correspondence_distance = 0.1;
};

/**
 * @brief
 *
 */
struct PlayerConfiguration {
  unsigned int entry_delay_ms =
      200; // delay inbetween processing filescan entries
};

/**
 * @brief SLAM Configuraton.
 *
 */
struct SlamConfiguration {
  bool with_imu = false;
  bool with_lidar = true;
  MapType map_type = MapType::KdTree;
  std::string remote_scanner = "local";

  SlamParameters parameters;
  PlayerConfiguration player_config;

  friend std::ostream &operator<<(std::ostream &os, SlamConfiguration config) {
    os << "IMU: " << config.with_imu << "\n"
       << "LIDAR: " << config.with_lidar << "\n"
       << "Map Type: "
       << (config.map_type == MapType::KdTree ? "KDtree" : "Voxel") << "\n"
       << "Scanner: " << config.remote_scanner << "\n"
       << "# SLAM Parameters #" << "\n\n"
       << "Optimizer Iterations: " << config.parameters.opt_iterations << "\n"
       << "Registration Iterations: " << config.parameters.reg_iterations
       << "\n"
       << "Max Corr. Distance: "
       << config.parameters.max_correspondence_distance << "\n"
       << "# #" << "\n"
       << config.parameters.max_correspondence_distance << "\n"
       << "Player Configuration" << "\n"
       << "Entry Delay: " << config.player_config.entry_delay_ms << "\n";
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