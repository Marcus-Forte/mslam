#pragma once

#include "ILog.hh"
#include <ostream>
#include <stdexcept>
#include <string>

namespace mslam {

enum class MapType { KdTree, Voxel };
enum class RegistrationMetric3D { PointToPoint, PointToPlane };

inline std::string toString(ILog::Level level) {
  switch (level) {
  case ILog::Level::DEBUG:
    return "DEBUG";
  case ILog::Level::INFO:
    return "INFO";
  case ILog::Level::WARNING:
    return "WARNING";
  case ILog::Level::ERROR:
    return "ERROR";
  }

  throw std::runtime_error("Invalid log level");
}

inline std::string toString(RegistrationMetric3D metric) {
  switch (metric) {
  case RegistrationMetric3D::PointToPoint:
    return "point_to_point";
  case RegistrationMetric3D::PointToPlane:
    return "point_to_plane";
  }

  throw std::runtime_error("Invalid 3D registration metric");
}

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
 * @brief sensor data preprocessor
 *
 */
struct PreProcessor {
  float voxel_size = 0.1;              // input scan voxel downsampling
  float min_distance_to_center = 0.0F; // remove points closer than this
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
  ILog::Level log_level = ILog::Level::INFO;
  MapType map_type = MapType::KdTree;
  std::string remote_scanner = "local";

  PreProcessor preprocessor;
  SlamParameters parameters;
  MapParameters map_parameters;

  friend std::ostream &operator<<(std::ostream &os, SlamConfiguration config) {
    os << "IMU: " << config.with_imu << "\n"
       << "LIDAR: " << config.with_lidar << "\n"
       << "Log Level: " << toString(config.log_level) << "\n"
       << "Scanner: " << config.remote_scanner << "\n"
       << "# MAP Parameters #" << "\n"
       << "Map Type: "
       << (config.map_type == MapType::KdTree ? "KDtree" : "Voxel") << "\n"
       << "Map resolution: " << config.map_parameters.resolution << "\n"
       << "Max pts per voxel: " << config.map_parameters.max_points_per_voxel
       << "\n"
       << "# Preprocessor Parameters #" << "\n"
       << "Voxel Size: " << config.preprocessor.voxel_size << "\n"
       << "Min Distance To Center: "
       << config.preprocessor.min_distance_to_center << "\n"
       << "# SLAM Parameters #" << "\n"
       << "Optimizer Iterations: " << config.parameters.opt_iterations << "\n"
       << "Registration Iterations: " << config.parameters.reg_iterations
       << "\n"
       << "Max Corr. Distance: "
       << config.parameters.max_correspondence_distance << "\n";
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