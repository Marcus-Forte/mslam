#include "config/JsonConfig.hh"
#include "config/Validation.hh"
#include "jsoncpp/json/reader.h"
#include <fstream>

namespace mslam {

namespace {
ILog::Level parseLogLevel(const std::string &log_level) {
  if (log_level == "trace") {
    return ILog::Level::TRACE;
  }
  if (log_level == "debug") {
    return ILog::Level::DEBUG;
  }
  if (log_level == "info") {
    return ILog::Level::INFO;
  }
  if (log_level == "warning") {
    return ILog::Level::WARNING;
  }
  if (log_level == "error") {
    return ILog::Level::ERROR;
  }

  throw std::runtime_error("Invalid log_level field: " + log_level);
}

mslam::DownsampleFilter parseDownsampleFilter(const std::string &filter) {
  if (filter == "voxel_grid") {
    return mslam::DownsampleFilter::VoxelGrid;
  }
  if (filter == "voxel_hash" || filter == "voxel_first_point") {
    return mslam::DownsampleFilter::VoxelHash;
  }

  throw std::runtime_error("Invalid preprocessor downsample_filter: " + filter);
}
} // namespace

JsonConfig::JsonConfig(const std::filesystem::path &config_file)
    : config_file_(config_file) {}

void JsonConfig::load() {
  Json::Value root;
  std::ifstream file(config_file_);
  file >> root;
  config_.with_imu = root["imu"].asBool();
  config_.with_lidar = root["lidar"].asBool();
  if (!root["log_level"].empty()) {
    config_.log_level = parseLogLevel(root["log_level"].asString());
  }
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
  // Preprocessor parameters

  if (root["preprocessor"]["voxel_size"].empty()) {
    throw std::runtime_error("Empty preprocessor voxel_size setting");
  }
  config_.preprocessor.voxel_size =
      root["preprocessor"]["voxel_size"].asFloat();
  if (!root["preprocessor"]["min_distance_to_center"].empty()) {
    config_.preprocessor.min_distance_to_center =
        root["preprocessor"]["min_distance_to_center"].asFloat();
  }
  if (!root["preprocessor"]["downsample_filter"].empty()) {
    config_.preprocessor.downsample_filter = parseDownsampleFilter(
        root["preprocessor"]["downsample_filter"].asString());
  }
  if (config_.preprocessor.min_distance_to_center < 0.0F) {
    throw std::runtime_error(
        "Invalid preprocessor min_distance_to_center setting");
  }

  // Map parameters
  if (root["map"]["resolution"].empty()) {
    throw std::runtime_error("Empty map resolution setting");
  }

  if (root["map"]["max_points_per_voxel"].empty()) {
    throw std::runtime_error("Empty map max_points_per_voxel setting");
  }
  config_.map_parameters.resolution = root["map"]["resolution"].asFloat();
  config_.map_parameters.max_points_per_voxel =
      root["map"]["max_points_per_voxel"].asUInt();

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
