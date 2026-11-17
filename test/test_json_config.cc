#include "mslam/config/JsonConfig.hh"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

// A complete, valid configuration; `slam_extra` is spliced into the "slam"
// object (e.g. `, "imu_fusion": "dead_reckoning"`).
std::string configText(const std::string &slam_extra,
                       const std::string &map_extra = "") {
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
    }
})";
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
                                const std::string &map_extra = "") {
    const auto path = dir_ / "config.jsonc";
    std::ofstream(path) << configText(slam_extra, map_extra);
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

TEST_F(JsonConfigTest, NumThreadsDefaultsToOneWhenAbsent) {
  EXPECT_EQ(load("").parameters.num_threads, 1U);
}

TEST_F(JsonConfigTest, NumThreadsParsesCountsAndZero) {
  EXPECT_EQ(load(R"(, "num_threads": 4)").parameters.num_threads, 4U);
  EXPECT_EQ(load(R"(, "num_threads": 0)").parameters.num_threads, 0U);
}

TEST_F(JsonConfigTest, NumThreadsRejectsAnInvalidValue) {
  EXPECT_THROW(load(R"(, "num_threads": "4")"), std::runtime_error);
  EXPECT_THROW(load(R"(, "num_threads": -1)"), std::runtime_error);
  EXPECT_THROW(load(R"(, "num_threads": 2.5)"), std::runtime_error);
  EXPECT_THROW(load(R"(, "num_threads": true)"), std::runtime_error);
  EXPECT_THROW(load(R"(, "num_threads": 257)"), std::runtime_error);
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
