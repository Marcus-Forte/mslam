#pragma once

#include "mslam/common/State.hh"

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <map>
#include <optional>
#include <span>
#include <spdlog/common.h>
#include <string>
#include <utility>
#include <vector>

namespace mslam {

/// Tuning of the `register_scans` tool (see `config/registration.jsonc`).
/// Plain fields only, so this library does not depend on the SLAM one; the
/// tool copies them into its registration parameters.
struct RegistrationConfig {
  spdlog::level::level_enum log_level = spdlog::level::info;
  float voxel_size = 0.05F;    ///< m, thinning of every input cloud.
  float map_resolution = 0.1F; ///< m, voxel size of the target map.
  std::uint32_t max_points_per_voxel = 8;
  int reg_iterations = 30;
  int optimizer_iterations = 5;
  float max_correspondence_distance = 0.3F; ///< m.
  /// Threads for the correspondence search and normal estimation (1:
  /// sequential, 0: every hardware thread). Results do not depend on it.
  unsigned num_threads = 1;
};

/// Reads a registration config (JSON with comments). Every key is optional.
/// \throws std::runtime_error if the file cannot be read or parsed, on an
/// unknown key (top level, "map" or "registration"), on a leftover "sycl" block
/// and on an invalid value; the message names the key.
RegistrationConfig loadRegistrationConfig(const std::filesystem::path &path);

/// Pose of a cloud's own frame in the common frame, by file name.
using PoseMap = std::map<std::string, Pose3D>;

/// Reads a poses file: a flat JSON object (comments allowed) whose keys are
/// input file names and whose values are `[x, y, z, roll, pitch, yaw]` (m, rad;
/// R = Rx(roll) * Ry(pitch) * Rz(yaw)).
/// \throws std::runtime_error if the file cannot be read or parsed, or a value
/// is not an array of exactly 6 finite numbers.
PoseMap loadPoseFile(const std::filesystem::path &path);

/// Matches the keys of @p poses to the input arguments, as typed. A key
/// matches an input if it equals the argument or its file-name component.
/// Returns one entry per input (empty if the input has no pose).
/// \throws std::runtime_error if a key matches no input or several, or if an
/// input is matched by several keys.
std::vector<std::optional<Pose3D>>
resolvePoses(const PoseMap &poses, std::span<const std::string> inputs);

/// Writes @p poses (name, pose) in the format loadPoseFile() reads (numbers
/// with enough digits to round trip) with a header comment stating the
/// convention.
/// \throws std::runtime_error if the stream fails or the file cannot be opened.
void savePoseFile(std::ostream &out,
                  std::span<const std::pair<std::string, Pose3D>> poses);
void savePoseFile(const std::filesystem::path &path,
                  std::span<const std::pair<std::string, Pose3D>> poses);

} // namespace mslam
