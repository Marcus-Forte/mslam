#include "config/JsonConfig.hh"
#include "jsoncpp/json/reader.h"
#include <fstream>

namespace mslam {
JsonConfig::JsonConfig(const std::filesystem::path &config_file)
    : config_file_(config_file) {}

void JsonConfig::load() {
  Json::Value root;
  std::ifstream file(config_file_);
  file >> root;
  config_.with_imu = root["imu"].asBool();
  config_.with_lidar = root["lidar"].asBool();
  const auto &map_type = root["map_type"].asString();
  if (map_type == "voxel") {
    config_.map_type = MapType::Voxel;
  } else if (map_type == "kdtree") {
    config_.map_type = MapType::KdTree;
  } else {
    throw std::runtime_error("Invalid map_type field: " + map_type);
  }
  config_.gl_server_address = root["gl_server_address"].asString();
  config_.remote_scan_address = root["remote_scan_address"].asString();
}
} // namespace mslam
