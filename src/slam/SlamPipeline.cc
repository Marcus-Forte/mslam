#include "mslam/slam/SlamPipeline.hh"

#include "mslam/map/VoxelHashMap.hh"
#include "mslam/slam/Transform.hh"

#include <utility>

namespace mslam {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kDenseMapVoxelSize = 0.01;
constexpr int kDenseMapVoxelBucketSize = 10;

std::chrono::nanoseconds since(Clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                              start);
}

} // namespace

SlamPipeline::SlamPipeline(Slam &slam, const SlamConfiguration &config,
                           std::shared_ptr<spdlog::logger> logger,
                           ISlamOutput &output)
    : slam_(slam), config_(config), logger_(std::move(logger)), output_(output),
      map_(slam.map()), preprocessor_(config.preprocessor, logger_),
      sync_(config.with_imu, config.preprocessor.points_per_second, logger_) {
  if (config_.map_parameters.dense_map) {
    dense_map_ = std::make_unique<VoxelHashMap>(kDenseMapVoxelSize,
                                                kDenseMapVoxelBucketSize);
  }
  logger_->debug("Using downsample filter: {}",
                 toString(config_.preprocessor.downsample_filter));
}

void SlamPipeline::publishPose() { output_.updatePose(slam_.getPose()); }

void SlamPipeline::pushImu(const msensor::IMUData &imu) { sync_.push(imu); }

void SlamPipeline::processSyncedScan(const Scan &scan) {
  sync_.take(scan, window_);
  for (const auto &imu : window_.samples) {
    processImu(imu);
  }
  if (window_.split.has_value()) {
    processImu(*window_.split);
  }
  processScan(scan, window_.samples);
}

void SlamPipeline::processImu(const msensor::IMUData &imu) {
  logger_->debug("Processing IMU @ {}, delta {} ms, seq nr {}, "
                 "raw_acc=[{}, {}, {}], raw_gyro=[{}, {}, {}]",
                 imu.header.timestamp,
                 (imu.header.timestamp - last_imu_time_ns_) * 1e-6,
                 imu.header.sequence_number, imu.ax, imu.ay, imu.az, imu.gx,
                 imu.gy, imu.gz);
  last_imu_time_ns_ = imu.header.timestamp;

  const auto start = Clock::now();
  slam_.Predict(imu);
  timings_.predict += since(start);
  publishPose();
}

void SlamPipeline::processScan(
    const Scan &scan, const std::vector<msensor::IMUData> &imu_samples) {
  ++timings_.scans;

  if (init_scan_count_ < kInitScans) {
    initScan(scan);
    return;
  }
  if (!config_.with_lidar) {
    return;
  }

  logger_->debug("Processing Lidar scan with {} points @ {}, seq nr {}",
                 scan.points.size(), scan.header.timestamp,
                 scan.header.sequence_number);

  auto start = Clock::now();
  preprocessor_.process(scan, imu_samples, filtered_scan_);
  timings_.preprocess += since(start);
  logger_->debug("Preprocess: {} -> {} pts", scan.points.size(),
                 filtered_scan_.points.size());

  start = Clock::now();
  slam_.Update(filtered_scan_);
  timings_.update += since(start);

  publishPose();

  start = Clock::now();
  transformCloud(slam_.getTransform(), filtered_scan_.points);
  insertAndPublish(filtered_scan_.points);
  timings_.map_insert += since(start);
}

void SlamPipeline::initScan(const Scan &scan) {
  const auto start = Clock::now();
  auto filtered_scan = preprocessor_.filterNearCenter(scan);
  timings_.preprocess += since(start);

  const auto insert_start = Clock::now();
  insertAndPublish(filtered_scan->points);
  timings_.map_insert += since(insert_start);
  ++init_scan_count_;

  if (logger_->should_log(spdlog::level::info)) {
    logger_->info("Init scan {}/{}. Map points: {}", init_scan_count_,
                  kInitScans, map_->getPointCloudRepresentation().size());
  }
  slam_.ResetImuPreintegration();
}

void SlamPipeline::insertAndPublish(PointCloud &points) {
  auto map_increment = map_->addScan(points);
  if (dense_map_) {
    dense_map_->addScan(points);
  }
  pruneMap();

  output_.updateTransformedScan(std::move(points));
  output_.updateMapIncrement(std::move(map_increment));
}

void SlamPipeline::pruneMap() {
  const auto pose = slam_.getPose();
  map_->prune(Point{static_cast<float>(pose[0]), static_cast<float>(pose[1]),
                    static_cast<float>(pose[2])},
              config_.map_parameters.max_range,
              config_.map_parameters.max_voxels);
}

void SlamPipeline::reset() {
  sync_.clear();
  slam_.ResetPose();
  map_->clear();
  if (dense_map_) {
    dense_map_->clear();
  }
  output_.reset();
  publishPose();
  logger_->info("SLAM reset: pose and map cleared");
}

} // namespace mslam
