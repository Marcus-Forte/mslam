#include "mslam/config/JsonConfig.hh"

#include <gtest/gtest.h>

#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace {

// A complete, valid configuration; `slam_extra` is spliced into the "slam"
// object (e.g. `, "imu_fusion": "dead_reckoning"`).
std::string configText(const std::string &slam_extra,
                       const std::string &map_extra = "",
                       const std::string &top_extra = "") {
  return R"({
    "imu": true,
    "lidar": true,
    "imu_acceleration_scale": 9.80665,
    "log_level": "info",
    "map_type": "voxel",
    "remote_scanner": "localhost:50051",
    "preprocessor": {
        "voxel_size": 0.1,
        "min_distance_to_center": 1.0,
        "min_intensity": 50,
        "downsample_filter": "voxel_hash",
        "deskew": true,
        "points_per_second": 200000
    },
    "map": { "resolution": 0.1, "max_points_per_voxel": 5)" +
         map_extra + R"( },
    "slam": {
        "optimizer_iterations": 3,
        "reg_iterations": 3,
        "max_correspondence_distance": 0.3)" +
         slam_extra + R"(
    })" + top_extra +
         "\n}";
}

class JsonConfigTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto *info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = std::filesystem::temp_directory_path() /
           (std::string("mslam_json_config_") + info->name());
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  mslam::SlamConfiguration load(const std::string &slam_extra,
                                const std::string &map_extra = "",
                                const std::string &top_extra = "") {
    const auto path = dir_ / "config.jsonc";
    std::ofstream(path) << configText(slam_extra, map_extra, top_extra);
    mslam::JsonConfig config(path);
    config.load();
    return config.getConfig();
  }

  std::filesystem::path dir_;
};

TEST_F(JsonConfigTest, ImuFusionDefaultsToPreintegrationFactorWhenAbsent) {
  EXPECT_EQ(load("").parameters.imu_fusion,
            mslam::ImuFusion::PreintegrationFactor);
}

TEST_F(JsonConfigTest, ImuFusionParsesBothValues) {
  EXPECT_EQ(
      load(R"(, "imu_fusion": "preintegration_factor")").parameters.imu_fusion,
      mslam::ImuFusion::PreintegrationFactor);
  EXPECT_EQ(load(R"(, "imu_fusion": "dead_reckoning")").parameters.imu_fusion,
            mslam::ImuFusion::DeadReckoning);
}

TEST_F(JsonConfigTest, ImuFusionRejectsAnInvalidValue) {
  EXPECT_THROW(load(R"(, "imu_fusion": "kalman")"), std::runtime_error);
  EXPECT_THROW(load(R"(, "imu_fusion": 1)"), std::runtime_error);
}

TEST_F(JsonConfigTest, StrategyDefaultsToOdometryWhenAbsent) {
  EXPECT_EQ(load("").strategy, mslam::SlamStrategy::Odometry);
}

TEST_F(JsonConfigTest, StrategyParsesBothValues) {
  EXPECT_EQ(load(R"(, "strategy": "odometry")").strategy,
            mslam::SlamStrategy::Odometry);
  EXPECT_EQ(load(R"(, "strategy": "pose_graph")").strategy,
            mslam::SlamStrategy::PoseGraph);
}

TEST_F(JsonConfigTest, StrategyRejectsAnInvalidValue) {
  EXPECT_THROW(load(R"(, "strategy": "icp")"), std::runtime_error);
  EXPECT_THROW(load(R"(, "strategy": 1)"), std::runtime_error);
}

TEST_F(JsonConfigTest, PoseGraphDefaultsWhenTheBlockIsAbsent) {
  const auto pg = load("").pose_graph;
  const mslam::PoseGraphParameters d;
  EXPECT_EQ(pg.keyframe_distance, 0.5);
  EXPECT_EQ(pg.keyframe_angle, 0.26);
  EXPECT_EQ(pg.keyframe_max_interval_s, 0.0);
  EXPECT_EQ(pg.mini_map_voxel_size, 0.05);
  EXPECT_EQ(pg.loop_search_radius, 2.0);
  EXPECT_EQ(pg.loop_min_separation, 10U);
  EXPECT_EQ(pg.loop_submap_keyframes, 2U);
  EXPECT_EQ(pg.loop_max_rms, 0.05);
  EXPECT_EQ(pg.loop_min_matched_fraction, 0.6);
  EXPECT_EQ(pg.optimizer_iterations, 20U);
  EXPECT_FALSE(pg.synchronous);
  EXPECT_EQ(pg.odometry_sigma_translation, d.odometry_sigma_translation);
}

TEST_F(JsonConfigTest, PoseGraphBlockParsesEveryKey) {
  const auto pg = load("", "", R"(, "pose_graph": {
      "keyframe_distance": 1.0, "keyframe_angle": 0.5,
      "keyframe_max_interval_s": 5.0, "mini_map_voxel_size": 0.1,
      "loop_search_radius": 3.0, "loop_min_separation": 20,
      "loop_submap_keyframes": 0, "loop_max_rms": 0.1,
      "loop_min_matched_fraction": 1.0, "loop_sigma_translation": 0.03,
      "loop_sigma_rotation": 0.006, "odometry_sigma_translation": 0.07,
      "odometry_sigma_rotation": 0.02, "optimizer_iterations": 7,
      "synchronous": true })")
                      .pose_graph;
  EXPECT_EQ(pg.keyframe_distance, 1.0);
  EXPECT_EQ(pg.keyframe_angle, 0.5);
  EXPECT_EQ(pg.keyframe_max_interval_s, 5.0);
  EXPECT_EQ(pg.mini_map_voxel_size, 0.1);
  EXPECT_EQ(pg.loop_search_radius, 3.0);
  EXPECT_EQ(pg.loop_min_separation, 20U);
  EXPECT_EQ(pg.loop_submap_keyframes, 0U);
  EXPECT_EQ(pg.loop_max_rms, 0.1);
  EXPECT_EQ(pg.loop_min_matched_fraction, 1.0);
  EXPECT_EQ(pg.loop_sigma_translation, 0.03);
  EXPECT_EQ(pg.loop_sigma_rotation, 0.006);
  EXPECT_EQ(pg.odometry_sigma_translation, 0.07);
  EXPECT_EQ(pg.odometry_sigma_rotation, 0.02);
  EXPECT_EQ(pg.optimizer_iterations, 7U);
  EXPECT_TRUE(pg.synchronous);
}

TEST_F(JsonConfigTest, PartialPoseGraphBlockKeepsTheOtherDefaults) {
  const auto pg =
      load("", "", R"(, "pose_graph": { "synchronous": true })").pose_graph;
  EXPECT_TRUE(pg.synchronous);
  EXPECT_EQ(pg.keyframe_distance, 0.5);
}

TEST_F(JsonConfigTest, PoseGraphRejectsOutOfRangeValues) {
  for (const char *bad : {R"("keyframe_distance": 0)",
                          R"("keyframe_distance": -1)",
                          R"("keyframe_angle": 0)",
                          R"("keyframe_angle": 4)",
                          R"("keyframe_max_interval_s": -1)",
                          R"("mini_map_voxel_size": 0)",
                          R"("loop_search_radius": 0)",
                          R"("loop_min_separation": 0)",
                          R"("loop_min_separation": 1.5)",
                          R"("loop_submap_keyframes": -1)",
                          R"("loop_max_rms": 0)",
                          R"("loop_min_matched_fraction": 0)",
                          R"("loop_min_matched_fraction": 1.5)",
                          R"("loop_sigma_translation": 0)",
                          R"("loop_sigma_rotation": -0.1)",
                          R"("odometry_sigma_translation": 0)",
                          R"("odometry_sigma_rotation": 0)",
                          R"("optimizer_iterations": 0)",
                          R"("keyframe_distance": "far")",
                          R"("synchronous": 1)"}) {
    EXPECT_THROW(
        load("", "", std::string(R"(, "pose_graph": { )") + bad + " }"),
        std::runtime_error)
        << bad;
  }
  EXPECT_THROW(load("", "", R"(, "pose_graph": 3)"), std::runtime_error);
}

TEST_F(JsonConfigTest, PoseGraphRejectsAnUnknownKeyAndNamesIt) {
  try {
    load("", "", R"(, "pose_graph": { "keyframe_distanse": 1.0 })");
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find("pose_graph.keyframe_distanse"),
              std::string::npos)
        << error.what();
  }
}

TEST(ShippedConfigs, LoadThroughJsonConfig) {
  const std::filesystem::path root = MSLAM_SOURCE_DIR;
  const auto load_file = [](const std::filesystem::path &path) {
    mslam::JsonConfig config(path);
    config.load();
    return config.getConfig();
  };
  // The owner edits config/mslam.jsonc for live runs (the strategy included),
  // so only require that it loads, whichever strategy it selects.
  EXPECT_NO_THROW(load_file(root / "config/mslam.jsonc"));
  EXPECT_EQ(load_file(root / "test/integration/mslam.jsonc").strategy,
            mslam::SlamStrategy::Odometry);
  const auto pose_graph =
      load_file(root / "test/integration/mslam_pose_graph.jsonc");
  EXPECT_EQ(pose_graph.strategy, mslam::SlamStrategy::PoseGraph);
  EXPECT_TRUE(pose_graph.pose_graph.synchronous);
}

// Sets MSLAM_SYCL for one test and restores the previous value.
class ScopedSyclEnv {
public:
  explicit ScopedSyclEnv(const char *value) {
    if (const char *old = std::getenv("MSLAM_SYCL")) {
      old_ = old;
    }
    if (value != nullptr) {
      setenv("MSLAM_SYCL", value, 1);
    } else {
      unsetenv("MSLAM_SYCL");
    }
  }
  ~ScopedSyclEnv() {
    if (old_.has_value()) {
      setenv("MSLAM_SYCL", old_->c_str(), 1);
    } else {
      unsetenv("MSLAM_SYCL");
    }
  }

private:
  std::optional<std::string> old_;
};

TEST_F(JsonConfigTest, SyclIsDisabledWhenTheBlockIsAbsent) {
  const ScopedSyclEnv env(nullptr);
  const auto config = load("");
  EXPECT_EQ(config.parameters.backend, mslam::ComputeBackend::Host);
  EXPECT_EQ(config.parameters.sycl_threads, 0U);
}

TEST_F(JsonConfigTest, SyclBlockParsesEnabledAndThreads) {
  const ScopedSyclEnv env(nullptr);
  auto config = load("", "", R"(, "sycl": {"enabled": true, "threads": 4})");
  EXPECT_EQ(config.parameters.backend, mslam::ComputeBackend::Sycl);
  EXPECT_EQ(config.parameters.sycl_threads, 4U);
  config = load("", "", R"(, "sycl": {"enabled": false, "threads": 0})");
  EXPECT_EQ(config.parameters.backend, mslam::ComputeBackend::Host);
  config = load("", "", R"(, "sycl": {})");
  EXPECT_EQ(config.parameters.backend, mslam::ComputeBackend::Host);
  EXPECT_EQ(config.parameters.sycl_threads, 0U);
}

TEST_F(JsonConfigTest, SyclBlockRejectsInvalidValues) {
  const ScopedSyclEnv env(nullptr);
  for (const char *block :
       {R"(, "sycl": true)", R"(, "sycl": {"enabled": 1})",
        R"(, "sycl": {"enabled": "yes"})", R"(, "sycl": {"threads": -1})",
        R"(, "sycl": {"threads": 2.5})", R"(, "sycl": {"threads": "4"})",
        R"(, "sycl": {"threads": 257})", R"(, "sycl": {"thread": 4})"}) {
    EXPECT_THROW(load("", "", block), std::runtime_error) << block;
  }
}

TEST_F(JsonConfigTest, RemovedKeysNameTheReplacement) {
  const ScopedSyclEnv env(nullptr);
  for (const char *key : {R"(, "num_threads": 4)", R"(, "backend": "sycl")"}) {
    try {
      load(key);
      ADD_FAILURE() << "accepted " << key;
    } catch (const std::runtime_error &error) {
      const std::string message = error.what();
      EXPECT_NE(message.find("was removed"), std::string::npos) << message;
      EXPECT_NE(message.find("sycl.enabled"), std::string::npos) << message;
      EXPECT_NE(message.find("sycl.threads"), std::string::npos) << message;
    }
  }
}

TEST_F(JsonConfigTest, MslamSyclOverridesTheFile) {
  {
    const ScopedSyclEnv env("1");
    EXPECT_EQ(load("").parameters.backend, mslam::ComputeBackend::Sycl);
  }
  {
    const ScopedSyclEnv env("0");
    EXPECT_EQ(load("", "", R"(, "sycl": {"enabled": true})").parameters.backend,
              mslam::ComputeBackend::Host);
  }
  {
    const ScopedSyclEnv env("maybe");
    EXPECT_THROW(load(""), std::runtime_error);
  }
}

TEST(SlamConfigurationPrint, ShowsTheSyclSettings) {
  mslam::SlamConfiguration config;
  std::ostringstream off;
  off << config;
  EXPECT_NE(off.str().find("SYCL: disabled"), std::string::npos);
  config.parameters.backend = mslam::ComputeBackend::Sycl;
  config.parameters.sycl_threads = 4;
  std::ostringstream on;
  on << config;
  EXPECT_NE(on.str().find("SYCL: enabled, threads 4"), std::string::npos);
}

TEST_F(JsonConfigTest, MapSearchRadiusDefaultsToOneWhenAbsent) {
  EXPECT_EQ(load("").map_parameters.search_radius, 1);
}

TEST_F(JsonConfigTest, MapSearchRadiusParsesValidValues) {
  EXPECT_EQ(load("", R"(, "search_radius": 0)").map_parameters.search_radius,
            0);
  EXPECT_EQ(load("", R"(, "search_radius": 3)").map_parameters.search_radius,
            3);
  EXPECT_EQ(load("", R"(, "search_radius": 8)").map_parameters.search_radius,
            8);
}

TEST_F(JsonConfigTest, MapSearchRadiusRejectsAnInvalidValue) {
  EXPECT_THROW(load("", R"(, "search_radius": -1)"), std::runtime_error);
  EXPECT_THROW(load("", R"(, "search_radius": 9)"), std::runtime_error);
  EXPECT_THROW(load("", R"(, "search_radius": 1.5)"), std::runtime_error);
  EXPECT_THROW(load("", R"(, "search_radius": 2.0)"), std::runtime_error);
  EXPECT_THROW(load("", R"(, "search_radius": "2")"), std::runtime_error);
}

} // namespace
