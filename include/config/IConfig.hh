#pragma once

#include <ostream>

namespace mslam {

enum class MapType { KdTree, Voxel };

struct Config {
  bool with_imu = false;
  bool with_lidar = false;
  MapType map_type = MapType::KdTree;
  std::string gl_server_address;
  std::string remote_scan_address;

  friend std::ostream &operator<<(std::ostream &os, Config config) {
    os << "IMU: " << config.with_imu << "\n"
       << "LIDAR: " << config.with_lidar << "\n"
       << "map type: "
       << (config.map_type == MapType::KdTree ? "KDtree" : "Voxel") << "\n"
       << "gl server ip: " << config.gl_server_address << "\n"
       << "remote scan ip: " << config.remote_scan_address << "\n";
    return os;
  }
};

class IConfig {
public:
  virtual void load() = 0;
  inline Config getConfig() const { return config_; }

protected:
  Config config_;
};

} // namespace mslam