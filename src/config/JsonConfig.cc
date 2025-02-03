#include "config/JsonConfig.hh"
#include "config/Validation.hh"
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
  config_.remote_scanner = root["remote_scanner"].asString();

  if (!isValidIPAndPort(config_.remote_scanner)) {
    throw std::runtime_error("Invalid Remote Scan IP: " +
                             config_.remote_scanner);
  }

  // Slam parameters
  config_.parameters.opt_iterations =
      root["slam"]["optimizer_iterations"].asUInt();
  config_.parameters.reg_iterations = root["slam"]["reg_iterations"].asUInt();
  config_.parameters.max_correspondence_distance =
      root["slam"]["max_correspondence_distance"].asFloat();

  if (config_.parameters.opt_iterations <= 0) {
    throw std::runtime_error("Invalid optimizer iterations: " +
                             std::to_string(config_.parameters.opt_iterations));
  }
}
} // namespace mslam
