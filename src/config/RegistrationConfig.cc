#include "mslam/config/RegistrationConfig.hh"

#include "jsoncpp/json/reader.h"
#include "jsoncpp/json/writer.h"
#include "mslam/config/IConfig.hh"
#include "mslam/config/Validation.hh"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <initializer_list>
#include <ostream>
#include <stdexcept>
#include <string_view>

namespace mslam {

namespace {

[[noreturn]] void fail(const std::string &what) {
  throw std::runtime_error(what);
}

// Parses a JSON document with comments whose root must be an object. Duplicate
// keys are rejected: the last one would silently win otherwise.
Json::Value parseObjectFile(const std::filesystem::path &path,
                            std::string_view what) {
  std::ifstream file(path);
  if (!file) {
    fail("cannot open " + std::string(what) + " " + path.string());
  }
  Json::CharReaderBuilder builder;
  builder["allowComments"] = true;
  builder["rejectDupKeys"] = true;
  Json::Value root;
  std::string errors;
  if (!Json::parseFromStream(builder, file, &root, &errors)) {
    fail("cannot parse " + std::string(what) + " " + path.string() + ": " +
         errors);
  }
  if (!root.isObject()) {
    fail(std::string(what) + " " + path.string() +
         ": the top level must be an object");
  }
  return root;
}

std::string qualified(std::string_view section, std::string_view key) {
  return section.empty() ? std::string(key)
                         : std::string(section) + "." + std::string(key);
}

void rejectUnknownKeys(const Json::Value &object, std::string_view section,
                       std::initializer_list<std::string_view> known) {
  mslam::rejectUnknownKeys(object, section, known, "registration config");
}

const Json::Value *optionalMember(const Json::Value &object,
                                  std::string_view key) {
  const std::string name(key);
  return object.isMember(name) ? &object[name] : nullptr;
}

// A finite number > 0 that is still finite and > 0 as a float.
float readPositiveFloat(const Json::Value &object, std::string_view section,
                        std::string_view key, float fallback) {
  const Json::Value *value = optionalMember(object, key);
  if (value == nullptr) {
    return fallback;
  }
  if (!value->isNumeric()) {
    fail("registration config: '" + qualified(section, key) +
         "' must be a number");
  }
  const double number = value->asDouble();
  const auto as_float = static_cast<float>(number);
  if (!std::isfinite(number) || !std::isfinite(as_float) || as_float <= 0.0F) {
    fail("registration config: '" + qualified(section, key) +
         "' must be finite and > 0");
  }
  return as_float;
}

// An integer in [min, max].
std::int64_t readInteger(const Json::Value &object, std::string_view section,
                         std::string_view key, std::int64_t fallback,
                         std::int64_t min, std::int64_t max) {
  const Json::Value *value = optionalMember(object, key);
  if (value == nullptr) {
    return fallback;
  }
  if (!value->isIntegral()) {
    fail("registration config: '" + qualified(section, key) +
         "' must be an integer");
  }
  const double number = value->asDouble();
  if (number < static_cast<double>(min) || number > static_cast<double>(max)) {
    fail("registration config: '" + qualified(section, key) + "' must be in [" +
         std::to_string(min) + ", " + std::to_string(max) + "]");
  }
  return value->asInt64();
}

// An integer in [1, max].
std::int64_t readCount(const Json::Value &object, std::string_view section,
                       std::string_view key, std::int64_t fallback,
                       std::int64_t max) {
  return readInteger(object, section, key, fallback, 1, max);
}

const Json::Value *optionalSection(const Json::Value &root,
                                   std::string_view key) {
  const Json::Value *section = optionalMember(root, key);
  if (section != nullptr && !section->isObject()) {
    fail("registration config: '" + std::string(key) + "' must be an object");
  }
  return section;
}

} // namespace

RegistrationConfig loadRegistrationConfig(const std::filesystem::path &path) {
  const Json::Value root = parseObjectFile(path, "registration config");
  // SYCL was removed: name the replacement instead of "unknown key".
  if (root.isMember("sycl")) {
    fail("registration config: the \"sycl\" block was removed: SYCL is gone; "
         "use registration.num_threads for threads");
  }
  rejectUnknownKeys(root, "",
                    {"log_level", "voxel_size", "map", "registration"});

  RegistrationConfig config;
  if (const auto *level = optionalMember(root, "log_level")) {
    if (!level->isString()) {
      fail("registration config: 'log_level' must be a string");
    }
    config.log_level = parseLogLevel(level->asString());
  }
  config.voxel_size =
      readPositiveFloat(root, "", "voxel_size", config.voxel_size);

  if (const auto *map = optionalSection(root, "map")) {
    rejectUnknownKeys(*map, "map", {"resolution", "max_points_per_voxel"});
    config.map_resolution =
        readPositiveFloat(*map, "map", "resolution", config.map_resolution);
    config.max_points_per_voxel = static_cast<std::uint32_t>(
        readCount(*map, "map", "max_points_per_voxel",
                  config.max_points_per_voxel, 1'000'000));
  }

  if (const auto *registration = optionalSection(root, "registration")) {
    rejectUnknownKeys(*registration, "registration",
                      {"reg_iterations", "optimizer_iterations",
                       "max_correspondence_distance", "num_threads"});
    config.reg_iterations = static_cast<int>(
        readCount(*registration, "registration", "reg_iterations",
                  config.reg_iterations, 1'000'000));
    config.optimizer_iterations = static_cast<int>(
        readCount(*registration, "registration", "optimizer_iterations",
                  config.optimizer_iterations, 1'000'000));
    config.max_correspondence_distance = readPositiveFloat(
        *registration, "registration", "max_correspondence_distance",
        config.max_correspondence_distance);
    config.num_threads = static_cast<unsigned>(
        readInteger(*registration, "registration", "num_threads",
                    config.num_threads, 0, 256));
  }
  return config;
}

PoseMap loadPoseFile(const std::filesystem::path &path) {
  const Json::Value root = parseObjectFile(path, "poses file");
  PoseMap poses;
  for (const auto &name : root.getMemberNames()) {
    const Json::Value &value = root[name];
    if (!value.isArray() || value.size() != 6) {
      fail("poses file " + path.string() + ": '" + name +
           "' must be an array of 6 numbers [x, y, z, roll, pitch, yaw]");
    }
    Pose3D pose;
    for (Json::ArrayIndex i = 0; i < 6; ++i) {
      if (!value[i].isNumeric() || !std::isfinite(value[i].asDouble())) {
        fail("poses file " + path.string() + ": '" + name + "' element " +
             std::to_string(i) + " must be a finite number");
      }
      pose(static_cast<Eigen::Index>(i)) = value[i].asDouble();
    }
    poses.emplace(name, pose);
  }
  return poses;
}

std::vector<std::optional<Pose3D>>
resolvePoses(const PoseMap &poses, std::span<const std::string> inputs) {
  std::vector<std::optional<Pose3D>> resolved(inputs.size());
  for (const auto &[key, pose] : poses) {
    std::optional<std::size_t> match;
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      const bool matches =
          key == inputs[i] ||
          key == std::filesystem::path(inputs[i]).filename().string();
      if (!matches) {
        continue;
      }
      if (match.has_value()) {
        fail("poses file: key '" + key + "' matches both '" + inputs[*match] +
             "' and '" + inputs[i] + "'");
      }
      match = i;
    }
    if (!match.has_value()) {
      fail("poses file: key '" + key + "' matches none of the input files");
    }
    if (resolved[*match].has_value()) {
      fail("poses file: input '" + inputs[*match] +
           "' has more than one pose (a key as typed and one by file name)");
    }
    resolved[*match] = pose;
  }
  return resolved;
}

void savePoseFile(std::ostream &out,
                  std::span<const std::pair<std::string, Pose3D>> poses) {
  out << "// Pose of each cloud's own frame in the common frame:\n"
         "// [x, y, z, roll, pitch, yaw] (m, rad; R = Rx(roll) * Ry(pitch) * "
         "Rz(yaw)).\n"
         "// Written by register_scans; it can be passed back as --poses.\n"
         "{\n";
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto &[name, pose] = poses[i];
    out << "    " << Json::valueToQuotedString(name.c_str()) << ": [";
    for (Eigen::Index k = 0; k < 6; ++k) {
      char number[40];
      std::snprintf(number, sizeof(number), "%.17g", pose(k));
      out << (k == 0 ? "" : ", ") << number;
    }
    out << "]" << (i + 1 < poses.size() ? "," : "") << "\n";
  }
  out << "}\n";
  out.flush();
  if (!out) {
    fail("failed to write the poses file");
  }
}

void savePoseFile(const std::filesystem::path &path,
                  std::span<const std::pair<std::string, Pose3D>> poses) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    fail("cannot open " + path.string() + " for writing");
  }
  savePoseFile(out, poses);
}

} // namespace mslam
