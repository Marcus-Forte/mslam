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
 * @brief sensor data preprocessor
 *
 */
struct PreProcessor {
  float voxel_size = 0.1; // input scan voxel downsampling
};

/**
 * @brief MAP Parameters
 *
 */
struct MapParameters {
  float resolution = 0.2;
  unsigned int max_points_per_voxel = 5;
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
  std::string remote_gl_server;

  PreProcessor preprocessor;
  SlamParameters parameters;
  PlayerConfiguration player_config;
  MapParameters map_parameters;

  friend std::ostream &operator<<(std::ostream &os, SlamConfiguration config) {
    os << "IMU: " << config.with_imu << "\n"
       << "LIDAR: " << config.with_lidar << "\n"
       << "Scanner: " << config.remote_scanner << "\n"
       << "GL Server: " << config.remote_gl_server << "\n"
       << "# MAP Parameters #" << "\n"
       << "Map Type: "
       << (config.map_type == MapType::KdTree ? "KDtree" : "Voxel") << "\n"
       << "Map resolution: " << config.map_parameters.resolution << "\n"
       << "Max pts per voxel: " << config.map_parameters.max_points_per_voxel
       << "\n"
       << "# Preprocessor Parameters #" << "\n"
       << "Voxel Size: " << config.preprocessor.voxel_size << "\n"
       << "# SLAM Parameters #" << "\n"
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