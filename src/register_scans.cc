// Point-to-plane registration of whole point clouds, in the order given.
//
//   register_scans [-c registration.jsonc] [--poses poses.jsonc]
//                  [-o merged.ply] [--save-poses out.jsonc]
//                  <a.ply|a.pbscan> <b> [<c> ...]
//
// Cloud 0 defines the common frame (identity, or its pose from --poses) and is
// not registered. Cloud i >= 1 is registered against a map holding ALL clouds
// registered so far, starting from its pose in --poses if listed there, else
// from the pose estimated for the previous cloud. The result is the pose of
// each cloud's own frame in the common frame.
//
// `.ply` inputs are read as they are; the points of every LiDAR scan of a
// `.pbscan` are concatenated raw (no SLAM, no deskew), so a recording must be
// static. See config/registration.jsonc and readme.md.
//
// One line per cloud goes to stdout; logging goes to stderr. The exit code is
// 1 on an error and 2 if a registration step found no correspondences (its
// pose is then the prior).
#include "mslam/config/RegistrationConfig.hh"
#include "mslam/slam/ComputeContext.hh"
#include "mslam/slam/PointCloudIO.hh"
#include "mslam/slam/ScanSequenceRegistration.hh"
#include "mslam/slam/Transform.hh"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

int usage(const char *program) {
  std::fprintf(
      stderr,
      "usage: %s [-c registration.jsonc] [--poses poses.jsonc] "
      "[-o merged.ply] [--save-poses out.jsonc] [--sycl|--no-sycl] "
      "<a.ply|a.pbscan> <b> [<c> ...]\n"
      "  Registers every cloud against all clouds before it (point to "
      "plane) and prints its pose in the frame of the first.\n"
      "  -c            tuning (config/registration.jsonc); defaults without\n"
      "  --poses       measured initial poses by file name; the others start "
      "from the previous estimate\n"
      "  -o            writes all clouds, full resolution, in the common "
      "frame\n"
      "  --save-poses  writes the poses in the --poses format\n"
      "  --sycl        run on SYCL (a build with MSLAM_WITH_SYCL); overrides "
      "sycl.enabled of the config\n"
      "  --no-sycl     run the sequential host code; overrides sycl.enabled\n",
      program);
  return 1;
}

struct Arguments {
  std::string config_path;
  std::string poses_path;
  std::string merged_path;
  std::string save_poses_path;
  std::optional<bool> sycl; ///< --sycl / --no-sycl: overrides sycl.enabled.
  std::vector<std::string> inputs;
};

// Returns false (after the caller prints the usage) on a malformed command
// line. Options may appear anywhere.
bool parseArguments(int argc, char **argv, Arguments &arguments) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    std::string *target = nullptr;
    if (arg == "-c") {
      target = &arguments.config_path;
    } else if (arg == "--poses") {
      target = &arguments.poses_path;
    } else if (arg == "-o") {
      target = &arguments.merged_path;
    } else if (arg == "--save-poses") {
      target = &arguments.save_poses_path;
    } else if (arg == "--sycl" || arg == "--no-sycl") {
      arguments.sycl = arg == "--sycl";
      continue;
    } else if (arg.size() > 1 && arg[0] == '-') {
      std::fprintf(stderr, "error: unknown option %s\n", argv[i]);
      return false;
    } else {
      arguments.inputs.emplace_back(arg);
      continue;
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "error: %s needs a value\n", argv[i]);
      return false;
    }
    *target = argv[++i];
  }
  if (arguments.sycl == true && !mslam::syclBackendAvailable()) {
    std::fprintf(stderr, "error: --sycl needs a build with "
                         "MSLAM_WITH_SYCL (e.g. the sycl preset)\n");
    return false;
  }
  if (arguments.inputs.size() < 2) {
    std::fprintf(stderr, "error: at least two input clouds are needed\n");
    return false;
  }
  return true;
}

void validateInputs(const std::vector<std::string> &inputs) {
  for (const auto &input : inputs) {
    const std::filesystem::path path(input);
    const auto extension = path.extension().string();
    if (extension != ".ply" && extension != ".pbscan") {
      throw std::runtime_error("unsupported input " + input +
                               " (expected .ply or .pbscan)");
    }
    if (!std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("input not found: " + input);
    }
  }
}

std::ofstream openForWriting(const std::string &path) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    throw std::runtime_error("cannot open " + path + " for writing");
  }
  return file;
}

void printStep(std::size_t index, const std::string &name,
               const mslam::SequenceStep &step) {
  std::printf("[%zu] %s read=%zu used=%zu ", index, name.c_str(),
              step.input_points, step.used_points);
  if (!step.registered) {
    std::printf("reference ");
  } else {
    const double percent = step.used_points == 0
                               ? 0.0
                               : 100.0 *
                                     static_cast<double>(step.matched_points) /
                                     static_cast<double>(step.used_points);
    std::printf("prior=%s matched=%zu (%.1f%%) rms=%.6f m ",
                step.prior_measured ? "measured" : "estimated",
                step.matched_points, percent, step.rms_m);
    if (step.first_round.valid) {
      std::printf("first_round=%.3f mm / %.3f mrad ",
                  1e3 * step.first_round.translation_m,
                  1e3 * step.first_round.rotation_rad);
    } else {
      std::printf("NO CORRESPONDENCES (prior kept) ");
    }
  }
  std::printf("pose=[%.9f %.9f %.9f %.9f %.9f %.9f]\n", step.pose.position.x(),
              step.pose.position.y(), step.pose.position.z(),
              step.pose.rotation.x(), step.pose.rotation.y(),
              step.pose.rotation.z());
}

int run(const Arguments &arguments) {
  // Everything that can fail cheaply fails before the first cloud is loaded.
  validateInputs(arguments.inputs);
  mslam::RegistrationConfig config;
  if (!arguments.config_path.empty()) {
    config = mslam::loadRegistrationConfig(arguments.config_path);
  }
  std::vector<std::optional<mslam::Pose3D>> measured(arguments.inputs.size());
  if (!arguments.poses_path.empty()) {
    measured = mslam::resolvePoses(mslam::loadPoseFile(arguments.poses_path),
                                   arguments.inputs);
  }
  std::ofstream merged_file;
  if (!arguments.merged_path.empty()) {
    merged_file = openForWriting(arguments.merged_path);
  }
  std::ofstream poses_file;
  if (!arguments.save_poses_path.empty()) {
    poses_file = openForWriting(arguments.save_poses_path);
  }

  auto logger = spdlog::stderr_color_mt("register_scans");
  logger->set_level(config.log_level);

  mslam::ScanSequenceParameters parameters;
  parameters.voxel_size = config.voxel_size;
  parameters.map_resolution = config.map_resolution;
  parameters.max_points_per_voxel = config.max_points_per_voxel;
  parameters.icp = {config.reg_iterations, config.optimizer_iterations,
                    config.max_correspondence_distance};
  const bool sycl = arguments.sycl.value_or(config.sycl_enabled);
  if (sycl && !mslam::syclBackendAvailable()) {
    throw std::runtime_error("sycl.enabled is true, but this build has no "
                             "SYCL (MSLAM_WITH_SYCL, e.g. the sycl preset)");
  }
  parameters.backend =
      sycl ? mslam::ComputeBackend::Sycl : mslam::ComputeBackend::Host;
  parameters.sycl_threads = config.sycl_threads;
  mslam::ScanSequenceRegistration sequence(parameters, logger);

  // Full-resolution clouds are only kept when they are written out.
  std::vector<mslam::PointCloud> placed_clouds;
  std::vector<std::pair<std::string, mslam::Pose3D>> poses;
  bool all_converged = true;
  for (std::size_t i = 0; i < arguments.inputs.size(); ++i) {
    mslam::PointCloud cloud = mslam::readPointCloud(arguments.inputs[i]);
    const auto step = sequence.add(cloud, measured[i]);
    printStep(i, arguments.inputs[i], step);
    all_converged =
        all_converged && (!step.registered || step.first_round.valid);

    mslam::Pose3D pose;
    pose << step.pose.position.transpose(), step.pose.rotation.transpose();
    poses.emplace_back(arguments.inputs[i], pose);
    if (merged_file.is_open()) {
      mslam::transformCloud(mslam::toAffine(step.pose), cloud);
      placed_clouds.push_back(std::move(cloud));
    }
  }

  if (merged_file.is_open()) {
    mslam::writePlyPointCloud(merged_file, placed_clouds);
  }
  if (poses_file.is_open()) {
    mslam::savePoseFile(poses_file, poses);
  }
  if (!all_converged) {
    std::fprintf(stderr, "error: a registration step found no "
                         "correspondences; its pose is its prior\n");
    return 2;
  }
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  Arguments arguments;
  if (!parseArguments(argc, argv, arguments)) {
    return usage(argv[0]);
  }
  try {
    return run(arguments);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "error: %s\n", error.what());
    return 1;
  }
}
