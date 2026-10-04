#include "config/JsonConfig.hh"
#include "config/Validation.hh"
#include "jsoncpp/json/reader.h"

#include <cmath>
#include <fstream>

namespace mslam {

namespace {
spdlog::level::level_enum parseLogLevel(const std::string &log_level) {
  if (log_level == "trace") {
    return spdlog::level::trace;
  }
  if (log_level == "debug") {
    return spdlog::level::debug;
  }
  if (log_level == "info") {
    return spdlog::level::info;
  }
  if (log_level == "warning") {
    return spdlog::level::warn;
  }
  if (log_level == "error") {
    return spdlog::level::err;
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

  if (root["imu"].empty()) {
    throw std::runtime_error("Empty imu setting");
  }
  if (root["lidar"].empty()) {
    throw std::runtime_error("Empty lidar setting");
  }
  config_.with_imu = root["imu"].asBool();
  config_.with_lidar = root["lidar"].asBool();
  if (root["imu_acceleration_scale"].empty()) {
    throw std::runtime_error("Empty imu_acceleration_scale setting");
  }
  config_.imu_acceleration_scale = root["imu_acceleration_scale"].asDouble();
  if (!std::isfinite(config_.imu_acceleration_scale) ||
      config_.imu_acceleration_scale <= 0.0) {
    throw std::runtime_error("Invalid imu_acceleration_scale setting");
  }

  if (root["log_level"].empty()) {
    throw std::runtime_error("Empty log_level setting");
  }
  config_.log_level = parseLogLevel(root["log_level"].asString());

  if (root["map_type"].empty()) {
    throw std::runtime_error("Empty map_type setting");
  }
  const auto &map_type = root["map_type"].asString();
  if (map_type == "voxel") {
    config_.map_type = MapType::Voxel;
  } else {
    throw std::runtime_error("Invalid map_type field: " + map_type);
  }
  config_.remote_scanner = root["remote_scanner"].asString();

  if (config_.remote_scanner.empty() ||
      !isValidIPAndPort(config_.remote_scanner)) {
    throw std::runtime_error("Invalid Remote Scan IP: " +
                             config_.remote_scanner);
  }
  // Preprocessor parameters

  if (root["preprocessor"]["voxel_size"].empty()) {
    throw std::runtime_error("Empty preprocessor voxel_size setting");
  }
  if (root["preprocessor"]["min_distance_to_center"].empty()) {
    throw std::runtime_error(
        "Empty preprocessor min_distance_to_center setting");
  }
  if (root["preprocessor"]["downsample_filter"].empty()) {
    throw std::runtime_error("Empty preprocessor downsample_filter setting");
  }
  if (root["preprocessor"]["deskew"].empty()) {
    throw std::runtime_error("Empty preprocessor deskew setting");
  }
  if (root["preprocessor"]["min_intensity"].empty()) {
    throw std::runtime_error("Empty preprocessor min_intensity setting");
  }
  config_.preprocessor.voxel_size =
      root["preprocessor"]["voxel_size"].asFloat();
  if (!std::isfinite(config_.preprocessor.voxel_size) ||
      config_.preprocessor.voxel_size <= 0.0F) {
    throw std::runtime_error("Invalid preprocessor voxel_size setting");
  }
  config_.preprocessor.min_distance_to_center =
      root["preprocessor"]["min_distance_to_center"].asFloat();
  config_.preprocessor.min_intensity =
      root["preprocessor"]["min_intensity"].asFloat();

  config_.preprocessor.downsample_filter = parseDownsampleFilter(
      root["preprocessor"]["downsample_filter"].asString());

  const auto &deskew_val = root["preprocessor"]["deskew"];
  if (deskew_val.isBool()) {
    config_.preprocessor.deskew_mode = deskew_val.asBool()
                                           ? mslam::DeskewMode::ConstantVelocity
                                           : mslam::DeskewMode::Off;
  } else {
    const auto deskew_str = deskew_val.asString();
    if (deskew_str == "off") {
      config_.preprocessor.deskew_mode = mslam::DeskewMode::Off;
    } else if (deskew_str == "constant_velocity") {
      config_.preprocessor.deskew_mode = mslam::DeskewMode::ConstantVelocity;
    } else {
      throw std::runtime_error("Invalid preprocessor deskew mode: " +
                               deskew_str);
    }
  }
  if (config_.preprocessor.min_distance_to_center < 0.0F) {
    throw std::runtime_error(
        "Invalid preprocessor min_distance_to_center setting");
  }

  if (root["preprocessor"]["points_per_second"].empty()) {
    throw std::runtime_error("Empty preprocessor points_per_second setting");
  }
  config_.preprocessor.points_per_second =
      root["preprocessor"]["points_per_second"].asUInt();

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
  if (!root["map"]["max_range"].empty()) {
    config_.map_parameters.max_range = root["map"]["max_range"].asFloat();
  }
  if (!root["map"]["max_voxels"].empty()) {
    config_.map_parameters.max_voxels = root["map"]["max_voxels"].asUInt();
  }
  if (!root["map"]["dense_map"].empty()) {
    config_.map_parameters.dense_map = root["map"]["dense_map"].asBool();
  }
  if (!std::isfinite(config_.map_parameters.resolution) ||
      config_.map_parameters.resolution <= 0.0F ||
      config_.map_parameters.max_points_per_voxel == 0 ||
      !std::isfinite(config_.map_parameters.max_range) ||
      config_.map_parameters.max_range < 0.0F) {
    throw std::runtime_error("Invalid map parameters");
  }

  // Slam parameters
  if (root["slam"]["optimizer_iterations"].empty()) {
    throw std::runtime_error("Empty slam optimizer_iterations setting");
  }
  if (root["slam"]["reg_iterations"].empty()) {
    throw std::runtime_error("Empty slam reg_iterations setting");
  }
  if (root["slam"]["max_correspondence_distance"].empty()) {
    throw std::runtime_error("Empty slam max_correspondence_distance setting");
  }
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
