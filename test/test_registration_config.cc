#include "mslam/config/RegistrationConfig.hh"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

class RegistrationConfigTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto *info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = std::filesystem::temp_directory_path() /
           (std::string("mslam_registration_config_") + info->name());
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::filesystem::path write(const std::string &name,
                              const std::string &text) {
    const auto path = dir_ / name;
    std::ofstream(path) << text;
    return path;
  }

  mslam::RegistrationConfig loadConfig(const std::string &text) {
    return mslam::loadRegistrationConfig(write("config.jsonc", text));
  }

  mslam::PoseMap loadPoses(const std::string &text) {
    return mslam::loadPoseFile(write("poses.jsonc", text));
  }

  // Expects loading to throw a runtime_error whose message contains @p key.
  void expectConfigError(const std::string &text, const std::string &key) {
    try {
      loadConfig(text);
      ADD_FAILURE() << "accepted: " << text;
    } catch (const std::runtime_error &error) {
      EXPECT_NE(std::string(error.what()).find(key), std::string::npos)
          << error.what();
    }
  }

  void expectPosesError(const std::string &text) {
    EXPECT_THROW(loadPoses(text), std::runtime_error) << text;
  }

  std::filesystem::path dir_;
};

void expectDefaults(const mslam::RegistrationConfig &config) {
  EXPECT_EQ(config.log_level, spdlog::level::info);
  EXPECT_FLOAT_EQ(config.voxel_size, 0.05F);
  EXPECT_FLOAT_EQ(config.map_resolution, 0.1F);
  EXPECT_EQ(config.max_points_per_voxel, 8U);
  EXPECT_EQ(config.reg_iterations, 30);
  EXPECT_EQ(config.optimizer_iterations, 5);
  EXPECT_FLOAT_EQ(config.max_correspondence_distance, 0.3F);
  EXPECT_FALSE(config.sycl_enabled);
  EXPECT_EQ(config.sycl_threads, 0U);
}

TEST_F(RegistrationConfigTest, DefaultsWithoutAFile) {
  expectDefaults(mslam::RegistrationConfig{});
}

TEST_F(RegistrationConfigTest, DefaultsFromAnEmptyObject) {
  expectDefaults(loadConfig("{}"));
}

TEST_F(RegistrationConfigTest, DefaultsForMissingKeys) {
  const auto config = loadConfig(R"({"voxel_size": 0.2, "map": {}})");
  EXPECT_FLOAT_EQ(config.voxel_size, 0.2F);
  EXPECT_FLOAT_EQ(config.map_resolution, 0.1F);
  EXPECT_EQ(config.max_points_per_voxel, 8U);
  EXPECT_EQ(config.reg_iterations, 30);
}

TEST_F(RegistrationConfigTest, ParsesEveryValueAndAcceptsComments) {
  const auto config = loadConfig(R"(
// leading comment
{
    "log_level": "warning", // trailing comment
    "voxel_size": 0.1,
    /* block comment */
    "map": { "resolution": 0.25, "max_points_per_voxel": 4 },
    "registration": {
        "reg_iterations": 12,
        "optimizer_iterations": 3,
        "max_correspondence_distance": 0.75
    }
})");
  EXPECT_EQ(config.log_level, spdlog::level::warn);
  EXPECT_FLOAT_EQ(config.voxel_size, 0.1F);
  EXPECT_FLOAT_EQ(config.map_resolution, 0.25F);
  EXPECT_EQ(config.max_points_per_voxel, 4U);
  EXPECT_EQ(config.reg_iterations, 12);
  EXPECT_EQ(config.optimizer_iterations, 3);
  EXPECT_FLOAT_EQ(config.max_correspondence_distance, 0.75F);
}

TEST_F(RegistrationConfigTest, TheShippedExampleParsesToTheDefaults) {
  expectDefaults(
      mslam::loadRegistrationConfig(std::filesystem::path(MSLAM_SOURCE_DIR) /
                                    "config" / "registration.jsonc"));
}

TEST_F(RegistrationConfigTest, RejectsUnknownKeysAtEveryLevel) {
  expectConfigError(R"({"voxel_sise": 0.1})", "voxel_sise");
  expectConfigError(R"({"map": {"resolutoin": 0.1}})", "map.resolutoin");
  expectConfigError(R"({"registration": {"iterations": 3}})",
                    "registration.iterations");
}

TEST_F(RegistrationConfigTest, RejectsInvalidValues) {
  expectConfigError(R"({"log_level": "verbose"})", "log_level");
  expectConfigError(R"({"log_level": 3})", "log_level");
  expectConfigError(R"({"voxel_size": "0.1"})", "voxel_size");
  expectConfigError(R"({"voxel_size": 0})", "voxel_size");
  expectConfigError(R"({"voxel_size": -0.1})", "voxel_size");
  expectConfigError(R"({"map": 3})", "map");
  expectConfigError(R"({"map": {"resolution": 0}})", "map.resolution");
  expectConfigError(R"({"map": {"resolution": -1}})", "map.resolution");
  expectConfigError(R"({"map": {"max_points_per_voxel": 0}})",
                    "map.max_points_per_voxel");
  expectConfigError(R"({"map": {"max_points_per_voxel": 2.5}})",
                    "map.max_points_per_voxel");
  expectConfigError(R"({"map": {"max_points_per_voxel": "8"}})",
                    "map.max_points_per_voxel");
  expectConfigError(R"({"registration": {"reg_iterations": 0}})",
                    "registration.reg_iterations");
  expectConfigError(R"({"registration": {"optimizer_iterations": 0}})",
                    "registration.optimizer_iterations");
  expectConfigError(R"({"registration": {"optimizer_iterations": 1.5}})",
                    "registration.optimizer_iterations");
  expectConfigError(R"({"registration": {"max_correspondence_distance": 0}})",
                    "registration.max_correspondence_distance");
  expectConfigError(
      R"({"registration": {"max_correspondence_distance": null}})",
      "registration.max_correspondence_distance");
  expectConfigError(R"({"sycl": {"threads": -1}})", "sycl.threads");
  expectConfigError(R"({"sycl": {"threads": 2.5}})", "sycl.threads");
  expectConfigError(R"({"sycl": {"threads": "4"}})", "sycl.threads");
  expectConfigError(R"({"sycl": {"threads": 257}})", "sycl.threads");
  expectConfigError(R"({"sycl": {"enabled": 1}})", "sycl.enabled");
  expectConfigError(R"({"sycl": {"thread": 1}})", "sycl.thread");
  expectConfigError(R"({"sycl": []})", "sycl");
}

TEST_F(RegistrationConfigTest, ParsesTheSyclBlock) {
  const auto config =
      loadConfig(R"({"sycl": {"enabled": true, "threads": 4}})");
  EXPECT_TRUE(config.sycl_enabled);
  EXPECT_EQ(config.sycl_threads, 4U);
  const auto all = loadConfig(R"({"sycl": {"threads": 0}})");
  EXPECT_FALSE(all.sycl_enabled);
  EXPECT_EQ(all.sycl_threads, 0U);
}

TEST_F(RegistrationConfigTest, RejectsTheRemovedNumThreadsKey) {
  expectConfigError(R"({"registration": {"num_threads": 4}})",
                    "registration.num_threads was removed");
}

TEST_F(RegistrationConfigTest, RejectsAnUnreadableOrUnparsableFile) {
  EXPECT_THROW(mslam::loadRegistrationConfig(dir_ / "missing.jsonc"),
               std::runtime_error);
  EXPECT_THROW(loadConfig("{ not json"), std::runtime_error);
  EXPECT_THROW(loadConfig("[1, 2]"), std::runtime_error);
  EXPECT_THROW(mslam::loadPoseFile(dir_ / "missing.jsonc"), std::runtime_error);
}

TEST_F(RegistrationConfigTest, ParsesAPosesFile) {
  const auto poses = loadPoses(R"(
// pose of each scan
{
    "b.ply":    [2.10, 0.00, -0.5, 0, 0.1, 1.5708],
    "c.pbscan": [1, 2, 3, 4, 5, 6] // not listed ones start from the estimate
})");
  ASSERT_EQ(poses.size(), 2U);
  const mslam::Pose3D b = poses.at("b.ply");
  EXPECT_DOUBLE_EQ(b(0), 2.10);
  EXPECT_DOUBLE_EQ(b(2), -0.5);
  EXPECT_DOUBLE_EQ(b(4), 0.1);
  EXPECT_DOUBLE_EQ(b(5), 1.5708);
  EXPECT_DOUBLE_EQ(poses.at("c.pbscan")(3), 4.0);
}

TEST_F(RegistrationConfigTest, RejectsMalformedPoses) {
  expectPosesError(R"({"a.ply": [1, 2, 3, 4, 5]})");
  expectPosesError(R"({"a.ply": [1, 2, 3, 4, 5, 6, 7]})");
  expectPosesError(R"({"a.ply": [1, 2, 3, 4, 5, "6"]})");
  expectPosesError(R"({"a.ply": [1, 2, 3, 4, 5, null]})");
  expectPosesError(R"({"a.ply": [1, 2, 3, 4, 5, 1e999]})");
  expectPosesError(R"({"a.ply": [1, 2, 3, 4, 5, NaN]})");
  expectPosesError(R"({"a.ply": "1 2 3 4 5 6"})");
  expectPosesError(
      R"({"a.ply": [1, 2, 3, 4, 5, 6], "a.ply": [0, 0, 0, 0, 0, 0]})");
  expectPosesError(R"([[1, 2, 3, 4, 5, 6]])");
}

TEST_F(RegistrationConfigTest, MatchesKeysByTheArgumentAsTypedOrByFileName) {
  const std::vector<std::string> inputs{"data/a.ply", "data/b.ply",
                                        "other/c.pbscan"};
  const auto poses = loadPoses(R"({
      "data/a.ply": [1, 0, 0, 0, 0, 0],
      "c.pbscan":   [0, 3, 0, 0, 0, 0]
  })");
  const auto resolved = mslam::resolvePoses(poses, inputs);
  ASSERT_EQ(resolved.size(), 3U);
  ASSERT_TRUE(resolved[0].has_value());
  EXPECT_DOUBLE_EQ((*resolved[0])(0), 1.0);
  EXPECT_FALSE(resolved[1].has_value());
  ASSERT_TRUE(resolved[2].has_value());
  EXPECT_DOUBLE_EQ((*resolved[2])(1), 3.0);
}

TEST_F(RegistrationConfigTest, RejectsAKeyMatchingNoInput) {
  const std::vector<std::string> inputs{"a.ply", "b.ply"};
  EXPECT_THROW(
      mslam::resolvePoses(loadPoses(R"({"z.ply": [0,0,0,0,0,0]})"), inputs),
      std::runtime_error);
}

TEST_F(RegistrationConfigTest, RejectsAKeyMatchingSeveralInputs) {
  // The file name b.ply matches both inputs.
  const std::vector<std::string> inputs{"x/b.ply", "y/b.ply", "a.ply"};
  EXPECT_THROW(
      mslam::resolvePoses(loadPoses(R"({"b.ply": [0,0,0,0,0,0]})"), inputs),
      std::runtime_error);
}

TEST_F(RegistrationConfigTest, RejectsAnInputWithTwoPoses) {
  const std::vector<std::string> inputs{"x/b.ply", "a.ply"};
  EXPECT_THROW(mslam::resolvePoses(loadPoses(R"({"x/b.ply": [0,0,0,0,0,0],
                                 "b.ply": [1,0,0,0,0,0]})"),
                                   inputs),
               std::runtime_error);
}

TEST_F(RegistrationConfigTest, SavedPosesLoadBackIdentically) {
  std::vector<std::pair<std::string, mslam::Pose3D>> saved;
  mslam::Pose3D first;
  first << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  mslam::Pose3D second;
  second << 1.0 / 3.0, -2.718281828459045, 1e-9, 0.1, -0.2, 3.141592653589793;
  mslam::Pose3D third;
  third << 123456.789012345678, -0.5, 5e-7, 1.0 / 7.0, 2.0 / 3.0, -1.0 / 9.0;
  saved.emplace_back("data/a.ply", first);
  saved.emplace_back("we \"ird\".pbscan", second);
  saved.emplace_back("c.ply", third);

  const auto path = dir_ / "out.jsonc";
  mslam::savePoseFile(path, saved);
  const auto loaded = mslam::loadPoseFile(path);

  ASSERT_EQ(loaded.size(), saved.size());
  for (const auto &[name, pose] : saved) {
    ASSERT_EQ(loaded.count(name), 1U) << name;
    EXPECT_TRUE(loaded.at(name) == pose) << name; // bit for bit
  }

  // The output is usable as --poses: keys are the arguments as typed.
  std::vector<std::string> inputs;
  for (const auto &entry : saved) {
    inputs.push_back(entry.first);
  }
  const auto resolved = mslam::resolvePoses(loaded, inputs);
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    ASSERT_TRUE(resolved[i].has_value());
    EXPECT_TRUE(*resolved[i] == saved[i].second);
  }

  // The header states the convention.
  std::ifstream in(path);
  std::string header;
  std::getline(in, header);
  EXPECT_EQ(header.rfind("//", 0), 0U);
}

TEST_F(RegistrationConfigTest, SavingToAnUnwritablePathThrows) {
  const std::vector<std::pair<std::string, mslam::Pose3D>> saved;
  EXPECT_THROW(mslam::savePoseFile(dir_ / "missing" / "out.jsonc", saved),
               std::runtime_error);
}

} // namespace
