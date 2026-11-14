// Deterministic offline replay benchmark.
//
// Reads a .pbscan recording synchronously and feeds it to the same
// SlamPipeline the live SlamRunner uses (IMU prediction, preprocessing,
// registration, map insertion), as fast as possible and with logging off. The
// result is reproducible run to run, unlike the timer-driven integration test,
// so it can be used to compare builds: the final pose must match to the digit
// for a pure refactor.
//
//   bench_slam <config.jsonc> <recording.pbscan> [repeats]
//
// Prints one line per run: the scan count, the total and per-stage wall time
// in milliseconds and the final pose. It is not registered with CTest.
#include "msensor/conversions/lidar_imu_conversions.hh"
#include "msensor/recorder/scan_player.hh"
#include "mslam/config/JsonConfig.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamOutput.hh"
#include "mslam/slam/SlamPipeline.hh"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double toMs(std::chrono::nanoseconds duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s config.jsonc recording.pbscan [repeats]\n",
                 argv[0]);
    return 1;
  }
  mslam::JsonConfig json_config(argv[1]);
  json_config.load();
  const auto config = json_config.getConfig();
  const int repeats = argc > 3 ? std::atoi(argv[3]) : 3;

  auto logger = spdlog::default_logger();
  logger->set_level(spdlog::level::off);

  for (int run = 0; run < repeats; ++run) {
    auto map = std::make_shared<mslam::VoxelHashMap>(
        config.map_parameters.resolution,
        config.map_parameters.max_points_per_voxel);
    map->setNumAdjacentVoxelSearch(1);
    mslam::Slam slam(logger, config, map);
    mslam::NullSlamOutput output;
    mslam::SlamPipeline pipeline(slam, config, logger, output);
    msensor::ScanPlayer player{std::filesystem::path(argv[2])};

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
  }
  return 0;
}
