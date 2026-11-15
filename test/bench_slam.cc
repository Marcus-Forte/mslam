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
//
// Prints one line per run: the scan count, the total and per-stage wall time
// in milliseconds and the final pose. It is not registered with CTest.
//
// --save-map writes the final accumulated map (every map increment the
// pipeline published, concatenated: what SlamServer::GetMap serves, not the
// bounded registration map) as a binary PLY. With repeats > 1 the map of the
// LAST run is written; the output file is opened before the first run, so an
// unwritable path fails immediately. The option may appear anywhere.
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "msensor/recorder/scan_player.hh"
#include "mslam/config/JsonConfig.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/PointCloudIO.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/SlamPipeline.hh"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

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
  void updateMapIncrement(mslam::PointCloud increment) override {
    chunks_.push_back(std::move(increment));
  }
  void reset() override { chunks_.clear(); }

  std::span<const mslam::PointCloud> chunks() const { return chunks_; }

private:
  std::vector<mslam::PointCloud> chunks_;
};

int usage(const char *program) {
  std::fprintf(stderr,
               "usage: %s config.jsonc recording.pbscan [repeats] "
               "[--save-map out.ply]\n"
               "  --save-map writes the final map (all published increments) "
               "of the last run\n",
               program);
  return 1;
}

} // namespace

int main(int argc, char **argv) {
  std::vector<const char *> positional;
  std::string save_map_path;
  bool save_map = false;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--save-map") {
      if (i + 1 >= argc) {
        return usage(argv[0]);
      }
      save_map_path = argv[++i];
      save_map = true;
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

  mslam::JsonConfig json_config(positional[0]);
  json_config.load();
  const auto config = json_config.getConfig();

  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);

  for (int run = 0; run < repeats; ++run) {
    auto map = std::make_shared<mslam::VoxelHashMap>(
        config.map_parameters.resolution,
        config.map_parameters.max_points_per_voxel);
    map->setNumAdjacentVoxelSearch(1);
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
    const auto drain = [&](bool flush) {
      while (!pending.empty() &&
             (flush || pipeline.imuCovers(pending.front()))) {
        pipeline.processSyncedScan(pending.front());
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
    std::printf("  registration: imu_path=%zu lidar_only=%zu | first round "
                "(n=%zu) mean %.3f mm / %.3f mrad | preintegrated %.3f s\n",
                stats.imu_scans, stats.lidar_only_scans,
                stats.first_round_scans, stats.meanFirstRoundTranslationMm(),
                stats.meanFirstRoundRotationMrad(),
                stats.preintegrated_seconds);
    const auto &sync = pipeline.syncStats();
    std::printf("  imu sync: scans=%zu uncovered=%zu splits=%zu "
                "stale_dropped=%zu overflow_dropped=%zu\n",
                sync.scans, sync.scans_uncovered, sync.splits,
                sync.stale_dropped, sync.overflow_dropped);

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
