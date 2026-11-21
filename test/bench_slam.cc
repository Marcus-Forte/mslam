// Deterministic offline replay benchmark.
//
// Reads a .pbscan recording synchronously and feeds it to the same
// SlamPipeline the live SlamRunner uses (IMU prediction, preprocessing,
// registration, map insertion), as fast as possible and with logging off. The
// result is reproducible run to run, unlike the timer-driven integration test,
// so it can be used to compare builds: the final pose must match to the digit
// for a pure refactor.
//
//   bench_slam <config.jsonc> <recording.pbscan> [repeats] [--save-map out.ply]
//              [--imu-fusion <preintegration_factor|dead_reckoning>]
//              [--trajectory out.csv] [--threads <n>]
//              [--strategy <odometry|pose_graph>]
//
// Prints one line per run: the scan count, the total and per-stage wall time
// in milliseconds and the final pose. It is not registered with CTest.
//
// --save-map writes the final accumulated map (every map increment the
// pipeline published, concatenated: what SlamServer::GetMap serves, not the
// bounded registration map) as a binary PLY. With repeats > 1 the map of the
// LAST run is written; the output file is opened before the first run, so an
// unwritable path fails immediately. The option may appear anywhere.
//
// --imu-fusion overrides slam.imu_fusion of the config, so one config serves
// both ways of using the IMU: the preintegration factor in the joint
// optimization, or dead reckoning that only seeds a pose-only registration.
//
// --trajectory writes a CSV with one row per registered scan (last run only,
// opened before the first run like --save-map): the scan timestamp (ns), the
// pose (x y z roll pitch yaw), velocity, gyro bias and accel bias after the
// scan, and the first registration round's correction in mm and mrad (empty
// when that round found no correspondences). tools: test/plot_imu_fusion.py.
//
// --threads overrides slam.num_threads of the config (1 sequential, 0 every
// hardware thread). The pose must not depend on it.
//
// --log-level shows the log at that level (the run is silent otherwise); with
// the pose graph each optimization logs its error in physical units at info.
//
// --strategy overrides slam.strategy. With pose_graph the run ends with
// SlamPipeline::finish() (the open keyframe joins the graph and the back end
// drains), and extra lines follow the usual ones: the back end's counters and
// the final pose in the odom frame (it must equal the odometry run's) and in
// the corrected map frame. --save-map then writes the keyframe-rendered map of
// that final result, last keyframe included.
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "msensor/recorder/scan_player.hh"
#include "mslam/config/JsonConfig.hh"
#include "mslam/map/Voxel.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/MapFactory.hh"
#include "mslam/slam/PointCloudIO.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/SlamPipeline.hh"
#include "mslam/slam/Transform.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tsl/robin_set.h>
#include <vector>

#include <spdlog/fmt/fmt.h>

namespace {

using Clock = std::chrono::steady_clock;

double toMs(std::chrono::nanoseconds duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}

/// Keeps every map increment, in order (what SlamServer::map_history_ holds).
/// The chunks are moved in, so the per-scan cost does not depend on the map
/// size. Pose and transformed scan are ignored.
class MapCollector final : public mslam::ISlamOutput {
public:
  void updatePose(const mslam::Pose3D &) override {}
  void updateTransformedScan(mslam::PointCloud) override {}
  using mslam::ISlamOutput::updateMapIncrement;
  void updateMapIncrement(mslam::PointCloud increment) override {
    chunks_.push_back(std::move(increment));
  }
  void reset() override { chunks_.clear(); }

  std::span<const mslam::PointCloud> chunks() const { return chunks_; }

private:
  std::vector<mslam::PointCloud> chunks_;
};

const char *const k_trajectory_header =
    "timestamp_ns,x,y,z,roll,pitch,yaw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,"
    "first_round_mm,first_round_mrad\n";

/// Appends the state after a scan and the first-round correction of that scan.
void appendTrajectoryRow(std::string &out, std::uint64_t timestamp_ns,
                         const mslam::Slam &slam) {
  const auto &s = slam.state();
  out += fmt::format(
      "{},{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},"
      "{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},{:.9f},",
      timestamp_ns, s.position.x(), s.position.y(), s.position.z(),
      s.rotation.x(), s.rotation.y(), s.rotation.z(), s.velocity.x(),
      s.velocity.y(), s.velocity.z(), s.gyro_bias.x(), s.gyro_bias.y(),
      s.gyro_bias.z(), s.accel_bias.x(), s.accel_bias.y(), s.accel_bias.z());
  if (const auto &first = slam.lastFirstRound(); first.valid) {
    out += fmt::format("{:.6f},{:.6f}\n", 1e3 * first.translation_m,
                       1e3 * first.rotation_rad);
  } else {
    out += ",\n";
  }
}

double distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return (a - b).norm();
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

/// Pose-graph strategy: the back end's counters and the final pose in both
/// frames.
void printPoseGraph(const mslam::PoseGraphResult &graph,
                    const mslam::SlamPipeline &pipeline,
                    const mslam::Slam &slam, const mslam::StageTimings &timings,
                    double voxel_size) {
  const auto &s = graph.stats;
  std::printf("  pose_graph: keyframes=%zu odometry_edges=%zu "
              "loop_attempts=%zu accepted=%zu rejected=%zu optimizations=%zu "
              "map_resends=%zu | backend=%.1f ms (optimizer %.1f ms)\n",
              s.keyframes, s.odometry_edges,
              s.candidates_accepted + s.candidates_rejected,
              s.candidates_accepted, s.candidates_rejected, s.optimizations,
              timings.map_resends, toMs(timings.backend),
              1e3 * s.optimizer_seconds);

  std::vector<double> gaps;
  std::vector<double> rms;
  std::vector<double> matched;
  for (const auto &edge : graph.edges) {
    if (edge.kind == mslam::EdgeKind::Loop) {
      gaps.push_back(static_cast<double>(edge.to - edge.from));
      rms.push_back(1e3 * edge.verify_rms);
      matched.push_back(100.0 * edge.verify_matched_fraction);
    }
  }
  if (!gaps.empty()) {
    const auto range = [](const std::vector<double> &v) {
      return std::make_pair(*std::min_element(v.begin(), v.end()),
                            *std::max_element(v.begin(), v.end()));
    };
    const auto [g0, g1] = range(gaps);
    const auto [r0, r1] = range(rms);
    const auto [m0, m1] = range(matched);
    std::printf("  loops: gap min/median/max=%.0f/%.0f/%.0f keyframes | rms "
                "%.1f/%.1f/%.1f mm | matched %.1f/%.1f/%.1f %%\n",
                g0, median(gaps), g1, r0, median(rms), r1, m0, median(matched),
                m1);
  }

  // Duplication of the published map: the keyframe clouds as StreamKeyframes
  // and the final map re-send publish them (what --save-map writes after
  // finish()), counted against the distinct map-frame voxels they fill.
  std::size_t published = 0;
  tsl::robin_set<mslam::Voxel3> voxels;
  const auto inverse = static_cast<float>(1.0 / voxel_size);
  for (const auto &cloud : graph.map_clouds) {
    published += cloud->size();
    for (const auto &point : *cloud) {
      voxels.insert(mslam::PointToVoxel(point, inverse));
    }
  }
  std::printf("  map: published=%zu points, distinct voxels=%zu (%.3f m), "
              "ratio=%.3f | keyframe views=%zu points\n",
              published, voxels.size(), voxel_size,
              voxels.empty() ? 0.0
                             : static_cast<double>(published) /
                                   static_cast<double>(voxels.size()),
              s.view_points);

  const Eigen::Affine3d odom = slam.getTransform();
  const Eigen::Affine3d map = pipeline.mapFromOdom() * odom;
  const mslam::Pose3D odom_pose = slam.getPose();
  const Eigen::Matrix3d map_rotation = map.linear();
  const Eigen::Vector3d map_euler = mslam::rotationToEuler(map_rotation);
  const Eigen::Vector3d map_position = map.translation();
  std::printf("  pose odom=[%.9f %.9f %.9f %.9f %.9f %.9f]\n"
              "  pose map =[%.9f %.9f %.9f %.9f %.9f %.9f]\n"
              "  distance odom-map=%.4f m | from origin: odom=%.4f m "
              "map=%.4f m\n",
              odom_pose[0], odom_pose[1], odom_pose[2], odom_pose[3],
              odom_pose[4], odom_pose[5], map_position.x(), map_position.y(),
              map_position.z(), map_euler.x(), map_euler.y(), map_euler.z(),
              distance(odom.translation(), map_position),
              odom.translation().norm(), map_position.norm());
}

int usage(const char *program) {
  std::fprintf(stderr,
               "usage: %s config.jsonc recording.pbscan [repeats] "
               "[--save-map out.ply] "
               "[--imu-fusion preintegration_factor|dead_reckoning] "
               "[--trajectory out.csv] [--threads n] "
               "[--strategy odometry|pose_graph] [--log-level level]\n"
               "  --save-map writes the final map (all published increments) "
               "of the last run\n"
               "  --imu-fusion overrides slam.imu_fusion of the config\n"
               "  --threads overrides slam.num_threads of the config "
               "(1 sequential, 0 all hardware threads)\n"
               "  --trajectory writes one CSV row per registered scan "
               "(last run)\n"
               "  --strategy overrides slam.strategy of the config\n"
               "  --log-level trace|debug|info|warning|error shows the log "
               "(default: off)\n",
               program);
  return 1;
}

} // namespace

int main(int argc, char **argv) {
  std::vector<const char *> positional;
  std::string save_map_path;
  bool save_map = false;
  std::string trajectory_path;
  bool save_trajectory = false;
  std::optional<mslam::ImuFusion> imu_fusion;
  std::optional<unsigned> num_threads;
  std::optional<mslam::SlamStrategy> strategy;
  std::optional<spdlog::level::level_enum> log_level;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--save-map") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      save_map_path = argv[++i];
      save_map = true;
    } else if (arg == "--trajectory") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      trajectory_path = argv[++i];
      save_trajectory = true;
    } else if (arg == "--imu-fusion") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      const std::string_view value = argv[++i];
      if (value == mslam::toString(mslam::ImuFusion::PreintegrationFactor)) {
        imu_fusion = mslam::ImuFusion::PreintegrationFactor;
      } else if (value == mslam::toString(mslam::ImuFusion::DeadReckoning)) {
        imu_fusion = mslam::ImuFusion::DeadReckoning;
      } else {
        std::fprintf(stderr, "error: invalid --imu-fusion %s\n", argv[i]);
        return usage(argv[0]);
      }
    } else if (arg == "--strategy") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      const std::string_view value = argv[++i];
      if (value == mslam::toString(mslam::SlamStrategy::Odometry)) {
        strategy = mslam::SlamStrategy::Odometry;
      } else if (value == mslam::toString(mslam::SlamStrategy::PoseGraph)) {
        strategy = mslam::SlamStrategy::PoseGraph;
      } else {
        std::fprintf(stderr, "error: invalid --strategy %s\n", argv[i]);
        return usage(argv[0]);
      }
    } else if (arg == "--log-level") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      try {
        log_level = mslam::parseLogLevel(argv[++i]);
      } catch (const std::exception &) {
        std::fprintf(stderr, "error: invalid --log-level %s\n", argv[i]);
        return usage(argv[0]);
      }
    } else if (arg == "--threads") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      const std::string_view value = argv[++i];
      unsigned parsed = 0;
      const auto [end, error] =
          std::from_chars(value.data(), value.data() + value.size(), parsed);
      if (error != std::errc{} || end != value.data() + value.size()) {
        std::fprintf(stderr, "error: invalid --threads %s\n", argv[i]);
        return usage(argv[0]);
      }
      num_threads = parsed;
    } else if (arg.size() > 1 && arg[0] == '-' && arg != "-") {
      // Unknown option (a negative repeat count is not meaningful either).
      return usage(argv[0]);
    } else {
      positional.push_back(argv[i]);
    }
  }
  if (positional.size() < 2 || positional.size() > 3) {
    return usage(argv[0]);
  }
  const char *recording_path = positional[1];
  const int repeats = positional.size() > 2 ? std::atoi(positional[2]) : 3;

  // Open the output first so a bad path fails before a long run.
  std::ofstream map_file;
  if (save_map) {
    map_file.open(save_map_path, std::ios::binary | std::ios::trunc);
    if (!map_file) {
      std::fprintf(stderr, "error: cannot open %s for writing\n",
                   save_map_path.c_str());
      return 1;
    }
  }

  std::ofstream trajectory_file;
  if (save_trajectory) {
    trajectory_file.open(trajectory_path, std::ios::trunc);
    if (!trajectory_file) {
      std::fprintf(stderr, "error: cannot open %s for writing\n",
                   trajectory_path.c_str());
      return 1;
    }
  }

  mslam::JsonConfig json_config(positional[0]);
  json_config.load();
  auto config = json_config.getConfig();
  if (imu_fusion.has_value()) {
    config.parameters.imu_fusion = *imu_fusion;
  }
  if (num_threads.has_value()) {
    config.parameters.num_threads = *num_threads;
  }
  if (strategy.has_value()) {
    config.strategy = *strategy;
  }

  auto logger = spdlog::default_logger();
  logger->set_level(log_level.value_or(spdlog::level::off));

  for (int run = 0; run < repeats; ++run) {
    auto map = mslam::makeVoxelHashMap(config.map_parameters);
    mslam::Slam slam(logger, config, map);
    mslam::NullSlamOutput null_output;
    MapCollector collector;
    mslam::ISlamOutput &output =
        save_map ? static_cast<mslam::ISlamOutput &>(collector)
                 : static_cast<mslam::ISlamOutput &>(null_output);
    mslam::SlamPipeline pipeline(slam, config, logger, output);
    msensor::ScanPlayer player{std::filesystem::path(recording_path)};

    // Scans wait in file order until the IMU stream has reached their end (or
    // the recording ends), exactly as the live loop holds them.
    std::deque<mslam::Scan> pending;
    // One row per registered scan, buffered per run so that only the last
    // run's rows reach the file.
    std::string trajectory_rows;
    const auto drain = [&](bool flush) {
      while (!pending.empty() &&
             (flush || pipeline.imuCovers(pending.front()))) {
        pipeline.processSyncedScan(pending.front());
        if (save_trajectory) {
          appendTrajectoryRow(trajectory_rows, pending.front().header.timestamp,
                              slam);
        }
        pending.pop_front();
      }
    };

    const auto wall_start = Clock::now();
    while (player.next()) {
      const auto &entry = player.getLastEntry();
      if (entry.entry_case() == sensors::RecordingEntry::kImu) {
        pipeline.pushImu(msensor::fromProtobuf(entry.imu()));
        drain(false);
      } else if (entry.entry_case() == sensors::RecordingEntry::kScan) {
        auto scan = msensor::fromProtobuf(entry.scan());
        if (scan.points.empty()) {
          continue;
        }
        pending.push_back(std::move(scan));
        drain(false);
      }
    }
    drain(true);
    pipeline.finish(); // pose graph only: seal the last keyframe, drain
    const double total_ms =
        toMs(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                                  wall_start));

    const auto &timings = pipeline.timings();
    const auto pose = slam.getPose();
    std::printf(
        "run %d scans=%zu total=%.1f ms | preprocess=%.1f register=%.1f "
        "map_insert=%.1f predict=%.1f | pose=[%.9f %.9f %.9f %.9f "
        "%.9f %.9f]\n",
        run, timings.scans, total_ms, toMs(timings.preprocess),
        toMs(timings.update), toMs(timings.map_insert), toMs(timings.predict),
        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    const auto &stats = slam.registrationStats();
    std::printf(
        "  registration: imu_path=%zu dead_reckoning=%zu "
        "lidar_only=%zu | first round "
        "(n=%zu) mean %.3f mm / %.3f mrad | preintegrated %.3f s\n",
        stats.imu_scans, stats.dead_reckoning_scans, stats.lidar_only_scans,
        stats.first_round_scans, stats.meanFirstRoundTranslationMm(),
        stats.meanFirstRoundRotationMrad(), stats.preintegrated_seconds);
    const auto &sync = pipeline.syncStats();
    std::printf("  imu sync: scans=%zu uncovered=%zu splits=%zu "
                "stale_dropped=%zu overflow_dropped=%zu\n",
                sync.scans, sync.scans_uncovered, sync.splits,
                sync.stale_dropped, sync.overflow_dropped);

    if (const auto graph = pipeline.backendResult()) {
      printPoseGraph(*graph, pipeline, slam, timings,
                     config.pose_graph.mini_map_voxel_size);
    }

    if (save_trajectory && run == repeats - 1) {
      trajectory_file << k_trajectory_header << trajectory_rows;
      trajectory_file.close();
      if (!trajectory_file) {
        std::fprintf(stderr, "error: writing %s failed\n",
                     trajectory_path.c_str());
        return 1;
      }
      std::printf("trajectory: %s rows=%zu\n", trajectory_path.c_str(),
                  static_cast<std::size_t>(std::count(
                      trajectory_rows.begin(), trajectory_rows.end(), '\n')));
    }

    if (save_map && run == repeats - 1) {
      std::size_t points = 0;
      for (const auto &chunk : collector.chunks()) {
        points += chunk.size();
      }
      try {
        const auto bytes =
            mslam::writePlyPointCloud(map_file, collector.chunks());
        map_file.close();
        std::printf("map: %s points=%zu size=%ju bytes\n",
                    save_map_path.c_str(), points,
                    static_cast<std::uintmax_t>(bytes));
      } catch (const std::exception &error) {
        std::fprintf(stderr, "error: writing %s: %s\n", save_map_path.c_str(),
                     error.what());
        return 1;
      }
    }
  }
  return 0;
}
