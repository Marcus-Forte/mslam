#include "mslam/config/JsonConfig.hh"

#include <cstdlib>
#include "jsoncpp/json/reader.h"
#include "mslam/config/Validation.hh"

#include <cmath>
#include <fstream>
#include <limits>

namespace mslam {

namespace {
mslam::DownsampleFilter parseDownsampleFilter(const std::string &filter) {
  if (filter == "voxel_grid") {
    return mslam::DownsampleFilter::VoxelGrid;
  }
  if (filter == "voxel_hash" || filter == "voxel_first_point") {
    return mslam::DownsampleFilter::VoxelHash;
  }

  throw std::runtime_error("Invalid preprocessor downsample_filter: " + filter);
}

mslam::ImuFusion parseImuFusion(const std::string &fusion) {
  if (fusion == "preintegration_factor") {
    return mslam::ImuFusion::PreintegrationFactor;
  }
  if (fusion == "dead_reckoning") {
    return mslam::ImuFusion::DeadReckoning;
  }

  throw std::runtime_error("Invalid slam imu_fusion: " + fusion);
}

mslam::SlamStrategy parseSlamStrategy(const std::string &strategy) {
  if (strategy == "odometry") {
    return mslam::SlamStrategy::Odometry;
  }
  if (strategy == "pose_graph") {
    return mslam::SlamStrategy::PoseGraph;
  }

  throw std::runtime_error("Invalid slam strategy: " + strategy);
}

// Optional numeric key of the pose_graph block: leaves `value` untouched when
// absent, throws unless the key is a number in (min, max] (or [min, max] when
// `min_inclusive`).
void readPoseGraphNumber(const Json::Value &block, const char *key,
                         double &value, double min, double max,
                         bool min_inclusive) {
  const auto &node = block[key];
  if (node.isNull()) {
    return;
  }
  const double v = node.isNumeric() ? node.asDouble() : std::nan("");
  if (!std::isfinite(v) || v > max || (min_inclusive ? v < min : v <= min)) {
    throw std::runtime_error(std::string("Invalid pose_graph ") + key +
                             " setting");
  }
  value = v;
}

void readPoseGraphCount(const Json::Value &block, const char *key,
                        unsigned int &value, unsigned int min,
                        unsigned int max) {
  const auto &node = block[key];
  if (node.isNull()) {
    return;
  }
  // Only a JSON integer token is a count (jsoncpp's isIntegral accepts 2.0).
  const bool is_integer_token =
      node.type() == Json::intValue || node.type() == Json::uintValue;
  if (!is_integer_token || node.asDouble() < min || node.asDouble() > max) {
    throw std::runtime_error(std::string("Invalid pose_graph ") + key +
                             " setting: expected an integer in [" +
                             std::to_string(min) + ", " + std::to_string(max) +
                             "]");
  }
  value = node.asUInt();
}

PoseGraphParameters parsePoseGraph(const Json::Value &block) {
  PoseGraphParameters pg;
  if (block.isNull()) {
    return pg;
  }
  if (!block.isObject()) {
    throw std::runtime_error("Invalid pose_graph setting: expected an object");
  }
  rejectUnknownKeys(
      block, "pose_graph",
      {"keyframe_distance", "keyframe_angle", "keyframe_max_interval_s",
       "mini_map_voxel_size", "loop_search_radius", "loop_min_separation",
       "loop_submap_keyframes", "loop_max_rms", "loop_min_matched_fraction",
       "loop_sigma_translation", "loop_sigma_rotation",
       "odometry_sigma_translation", "odometry_sigma_rotation",
       "optimizer_iterations", "synchronous"},
      "config");
  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kPi = 3.14159265358979323846;
  readPoseGraphNumber(block, "keyframe_distance", pg.keyframe_distance, 0.0,
                      kInf, false);
  readPoseGraphNumber(block, "keyframe_angle", pg.keyframe_angle, 0.0, kPi,
                      false);
  readPoseGraphNumber(block, "keyframe_max_interval_s",
                      pg.keyframe_max_interval_s, 0.0, kInf, true);
  readPoseGraphNumber(block, "mini_map_voxel_size", pg.mini_map_voxel_size, 0.0,
                      kInf, false);
  readPoseGraphNumber(block, "loop_search_radius", pg.loop_search_radius, 0.0,
                      kInf, false);
  readPoseGraphCount(block, "loop_min_separation", pg.loop_min_separation, 1,
                     1000000);
  readPoseGraphCount(block, "loop_submap_keyframes", pg.loop_submap_keyframes,
                     0, 1000);
  readPoseGraphNumber(block, "loop_max_rms", pg.loop_max_rms, 0.0, kInf, false);
  readPoseGraphNumber(block, "loop_min_matched_fraction",
                      pg.loop_min_matched_fraction, 0.0, 1.0, false);
  readPoseGraphNumber(block, "loop_sigma_translation",
                      pg.loop_sigma_translation, 0.0, kInf, false);
  readPoseGraphNumber(block, "loop_sigma_rotation", pg.loop_sigma_rotation, 0.0,
                      kInf, false);
  readPoseGraphNumber(block, "odometry_sigma_translation",
                      pg.odometry_sigma_translation, 0.0, kInf, false);
  readPoseGraphNumber(block, "odometry_sigma_rotation",
                      pg.odometry_sigma_rotation, 0.0, kInf, false);
  readPoseGraphCount(block, "optimizer_iterations", pg.optimizer_iterations, 1,
                     10000);
  if (!block["synchronous"].isNull()) {
    if (!block["synchronous"].isBool()) {
      throw std::runtime_error(
          "Invalid pose_graph synchronous setting: expected on/off");
    }
    pg.synchronous = block["synchronous"].asBool();
  }
  return pg;
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

  if (!root["preprocessor"]["deskew"].isBool()) {
    throw std::runtime_error(
        "Invalid preprocessor deskew setting: expected on/off");
  }
  config_.preprocessor.deskew = root["preprocessor"]["deskew"].asBool();
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
  if (!root["map"]["search_radius"].empty()) {
    const auto &search_radius = root["map"]["search_radius"];
    // jsoncpp's isInt()/isIntegral() accept 2.0; only a JSON integer token
    // (intValue/uintValue) is a valid radius.
    const bool is_integer_token = search_radius.type() == Json::intValue ||
                                  search_radius.type() == Json::uintValue;
    if (!is_integer_token || search_radius.asDouble() < 0.0 ||
        search_radius.asDouble() > MapParameters::kMaxSearchRadius) {
      throw std::runtime_error(
          "Invalid map search_radius: expected an integer in [0, " +
          std::to_string(MapParameters::kMaxSearchRadius) + "]");
    }
    config_.map_parameters.search_radius = search_radius.asInt();
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

  if (!root["slam"]["imu_fusion"].empty()) {
    if (!root["slam"]["imu_fusion"].isString()) {
      throw std::runtime_error("Invalid slam imu_fusion: expected a string");
    }
    config_.parameters.imu_fusion =
        parseImuFusion(root["slam"]["imu_fusion"].asString());
  }

  if (!root["slam"]["strategy"].empty()) {
    if (!root["slam"]["strategy"].isString()) {
      throw std::runtime_error("Invalid slam strategy: expected a string");
    }
    config_.strategy = parseSlamStrategy(root["slam"]["strategy"].asString());
  }
  config_.pose_graph = parsePoseGraph(root["pose_graph"]);

  // Parallelism is SYCL only: say so for the keys of the host thread era.
  for (const char *key : {"num_threads", "backend"}) {
    if (root["slam"].isMember(key)) {
      throw std::runtime_error(
          std::string("slam.") + key +
          " was removed: SYCL is the only parallel path, use sycl.enabled and "
          "sycl.threads (top-level \"sycl\" block)");
    }
  }
  const SyclSettings sycl = parseSyclSettings(root["sycl"], "config");
  config_.parameters.backend =
      sycl.enabled ? ComputeBackend::Sycl : ComputeBackend::Host;
  config_.parameters.sycl_threads = sycl.threads;
  // MSLAM_SYCL=0|1 overrides sycl.enabled, so one config (and one test or
  // benchmark) runs on either path.
  if (const char *override_enabled = std::getenv("MSLAM_SYCL")) {
    const std::string_view value(override_enabled);
    if (value != "0" && value != "1") {
      throw std::runtime_error("Invalid MSLAM_SYCL: " + std::string(value) +
                               " (expected 0 or 1)");
    }
    config_.parameters.backend =
        value == "1" ? ComputeBackend::Sycl : ComputeBackend::Host;
  }

  if (config_.parameters.opt_iterations <= 0) {
    throw std::runtime_error("Invalid optimizer iterations: " +
                             std::to_string(config_.parameters.opt_iterations));
  }
}
} // namespace mslam
