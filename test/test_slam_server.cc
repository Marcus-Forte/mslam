#include "mslam/slam/SlamServer.hh"

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

namespace {

mslam::PointCloud cloudOf(std::size_t count) {
  mslam::PointCloud cloud;
  for (std::size_t i = 0; i < count; ++i) {
    cloud.emplace_back(static_cast<float>(i), 0.0F, 0.0F, 1.0F);
  }
  return cloud;
}

class FakeControl final : public mslam::ISlamControl {
public:
  void startProcessing() override { ++starts; }
  void stopProcessing() override { ++stops; }
  void requestReset() override { ++resets; }
  int starts = 0, stops = 0, resets = 0;
};

class SlamServerTest : public ::testing::Test {
protected:
  SlamServerTest() : server_(spdlog::default_logger()) {}

  // The generated service interface is public on the base class.
  sensors::SlamService::Service &service() { return server_; }

  int mapPointCount() {
    sensors::Empty request;
    sensors::PointCloud3 response;
    EXPECT_TRUE(service().GetMap(nullptr, &request, &response).ok());
    return response.x_size();
  }

  mslam::SlamServer server_;
};

TEST_F(SlamServerTest, GetMapAccumulatesTheIncrements) {
  EXPECT_EQ(mapPointCount(), 0);
  server_.updateMapIncrement(cloudOf(3));
  server_.updateMapIncrement(cloudOf(4));
  EXPECT_EQ(mapPointCount(), 7);
}

TEST_F(SlamServerTest, ResetDropsTheAccumulatedMap) {
  server_.updateMapIncrement(cloudOf(3));
  ASSERT_EQ(mapPointCount(), 3);
  server_.reset();
  EXPECT_EQ(mapPointCount(), 0);
  // Increments published after the reset start a new history.
  server_.updateMapIncrement(cloudOf(2));
  EXPECT_EQ(mapPointCount(), 2);
}

TEST_F(SlamServerTest, ControlRpcsAreForwardedToTheControl) {
  sensors::Empty request;
  sensors::Empty response;

  // No control attached: the RPCs are accepted and ignored.
  EXPECT_TRUE(service().Start(nullptr, &request, &response).ok());

  FakeControl control;
  server_.setControl(&control);
  EXPECT_TRUE(service().Start(nullptr, &request, &response).ok());
  EXPECT_TRUE(service().Stop(nullptr, &request, &response).ok());
  EXPECT_TRUE(service().Reset(nullptr, &request, &response).ok());
  EXPECT_EQ(control.starts, 1);
  EXPECT_EQ(control.stops, 1);
  // The server only asks; the SLAM thread performs the reset.
  EXPECT_EQ(control.resets, 1);

  server_.setControl(nullptr);
  EXPECT_TRUE(service().Reset(nullptr, &request, &response).ok());
  EXPECT_EQ(control.resets, 1);
}

} // namespace
