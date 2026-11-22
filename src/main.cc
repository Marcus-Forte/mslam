#include "mslam/config/JsonConfig.hh"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "msensor/config/config.hh"
#include "msensor/lidar/mid360.hh"
#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/MapFactory.hh"
#include "mslam/slam/SensorStatus.hh"
#include "mslam/slam/Slam.hh"
#include "mslam/slam/SlamPipeline.hh"
#include "mslam/slam/SlamRunner.hh"
#include "mslam/slam/SlamServer.hh"
#include "sensors_remote_client.hh"
#include <atomic>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

namespace {

// Queue depths requested from a remote sensor server (msensor's
// `max_queued_samples`). The IMU depth matches SensorInput's own IMU buffer, so
// the server never drops what mslam would keep. SensorInput keeps only the
// newest scan, so a deeper scan queue would be discarded on arrival.
constexpr uint32_t k_remote_imu_queue = 1000;
constexpr uint32_t k_remote_scan_queue = 5;

// Sensor link reported by the GetStatus RPC for a remote sensor server.
class RemoteSensorStatus final : public mslam::ISensorStatus {
public:
  explicit RemoteSensorStatus(std::shared_ptr<msensor::SensorsRemoteClient> c)
      : client_(std::move(c)) {}

  mslam::SensorStatus sensorStatus() const override {
    const auto scan_age = client_->lastScanAge();
    const auto imu_age = client_->lastImuAge();
    return {.link =
                mslam::classifySensorLink(client_->linkUp(), scan_age, imu_age),
            .address = client_->address(),
            .scan_age = scan_age,
            .imu_age = imu_age};
  }

private:
  std::shared_ptr<msensor::SensorsRemoteClient> client_;
};

// An on-board sensor has no link to report.
class LocalSensorStatus final : public mslam::ISensorStatus {
public:
  mslam::SensorStatus sensorStatus() const override {
    return {.link = mslam::SensorLink::NotApplicable, .address = "local"};
  }
};

// Set from the signal handler, polled by the SLAM loop.
std::atomic<bool> g_stop_requested{false};
// Host-only check: this TU is also compiled by the SYCL device pass (it links
// moptim through `slam`), whose target does not report lock-free atomics.
#ifndef __SYCL_DEVICE_ONLY__
static_assert(std::atomic<bool>::is_always_lock_free,
              "the stop flag is written from a signal handler");
#endif

void onTerminationSignal(int /*signal_number*/) {
  g_stop_requested.store(true);
}

void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name << " [-c config.json] [-h]\n"
            << "  -c <file>  Load SLAM configuration from JSON\n"
            << "  -h         Show this help message\n";
}
} // namespace

int main(int argc, char **argv) {
  int opt;
  mslam::SlamConfiguration config;
  std::filesystem::path slam_config_path;
  while ((opt = getopt(argc, argv, "c:h")) != -1) {
    switch (opt) {
    case 'c': {
      std::cout << "Using config: " << optarg << std::endl;
      slam_config_path = optarg;
      mslam::JsonConfig json_config(optarg);
      json_config.load();
      config = json_config.getConfig();
      break;
    }
    case 'h':
      printUsage(argv[0]);
      return 0;

    case '?':
      printUsage(argv[0]);
      return 1;
      break;
    }
  }

  std::cout << config << std::endl;

  // Create logger.
  const auto spdlog_logger = std::make_shared<spdlog::logger>(
      "mslam", std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  spdlog_logger->set_level(config.log_level);
  const auto logger = spdlog_logger;

  // Create Map interface.
  std::shared_ptr<mslam::IMap> map;
  map = mslam::makeVoxelHashMap(config.map_parameters,
                                config.parameters.backend);

  mslam::SlamServer slam_server(logger);
  slam_server.start();

  // Create sensor readers.
  std::shared_ptr<msensor::ILidar> lidar_sensor;
  std::shared_ptr<msensor::IImu> imu_sensor;

  if (config.remote_scanner == "local") {
    auto sensor_config_path =
        slam_config_path.empty()
            ? std::filesystem::path("config/publisher_config.json")
            : slam_config_path.parent_path() / "publisher_config.json";
    if (!std::filesystem::exists(sensor_config_path)) {
      sensor_config_path = msensor::Config::defaultConfigPath();
    }
    auto sensor_config = msensor::Config::fromFile(sensor_config_path);
    if (!sensor_config.mid360.enable) {
      throw std::runtime_error(
          "Local mode requires mid360.enable in the msensor publisher config");
    }
    if (sensor_config.mid360.config.empty()) {
      throw std::runtime_error(
          "Local mode requires mid360.config in the msensor publisher config");
    }

    std::filesystem::path mid360_config_path(sensor_config.mid360.config);
    if (mid360_config_path.is_relative()) {
      mid360_config_path =
          sensor_config_path.parent_path() / mid360_config_path;
    }
    if (!std::filesystem::exists(mid360_config_path)) {
      throw std::runtime_error("Mid360 config file does not exist: " +
                               mid360_config_path.string());
    }

    auto mid360 = std::make_shared<msensor::Mid360>(
        mid360_config_path.string(),
        sensor_config.mid360.accumulate_scan_count);
    mid360->init();
    mid360->setMode(msensor::Mid360::Mode::Normal);
    mid360->setScanPattern(msensor::Mid360::ScanPattern::NonRepetitive);
    mid360->startSampling();
    lidar_sensor = mid360;
    imu_sensor = mid360;
    slam_server.setSensorStatus(std::make_shared<LocalSensorStatus>());
    logger->info("Initialized local Mid360 using config: {}",
                 mid360_config_path.string());

  } else {
    auto remote = std::make_shared<msensor::SensorsRemoteClient>(
        config.remote_scanner,
        msensor::RemoteClientOptions{.scan_queue_size = k_remote_scan_queue,
                                     .imu_queue_size = k_remote_imu_queue});
    remote->init();
    remote->start();
    lidar_sensor = remote;
    imu_sensor = remote;
    slam_server.setSensorStatus(std::make_shared<RemoteSensorStatus>(remote));
    // start() only launches the reader threads (gRPC connects lazily and keeps
    // retrying), so whether the link is up is what GetStatus reports.
    logger->info("Connecting to remote sensor server at {}",
                 config.remote_scanner);
  }

  mslam::Slam slam(logger, config, map);
  mslam::SlamPipeline pipeline(slam, config, logger, slam_server);
  mslam::SlamRunner runner(pipeline, lidar_sensor, imu_sensor, config, logger);
  slam_server.setControl(&runner);

  std::signal(SIGINT, onTerminationSignal);
  std::signal(SIGTERM, onTerminationSignal);

  runner.run(g_stop_requested);

  // The runner is about to be destroyed: detach it from the server.
  slam_server.setControl(nullptr);
  slam_server.setSensorStatus(nullptr);
  return 0;
}
