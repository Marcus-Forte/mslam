#include "mslam/slam/SlamServer.hh"

#include <atomic>
#include <chrono>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

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

class FakeSensorStatus final : public mslam::ISensorStatus {
public:
  mslam::SensorStatus sensorStatus() const override { return status; }
  mslam::SensorStatus status;
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

// ---------------------------------------------------------------------------
// StreamMap, through a real server and stub.

using Clock = std::chrono::steady_clock;
using Chunk = sensors::MapStreamChunk;

// Increment of `count` points whose x values are first, first + 1, ...
mslam::PointCloud runOf(std::size_t count, std::size_t first) {
  mslam::PointCloud cloud;
  cloud.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    cloud.emplace_back(static_cast<float>(first + i), 0.0F, 0.0F, 1.0F);
  }
  return cloud;
}

// A server on a unix socket plus a stub connected to it.
class RunningServer {
public:
  RunningServer()
      : address_(address()), server(spdlog::default_logger(), address_) {
    server.start();
    stub = sensors::SlamService::NewStub(
        grpc::CreateChannel(address_, grpc::InsecureChannelCredentials()));
  }
  ~RunningServer() { server.stop(); }

  // Publishes `count` increments of `size` points (x continues the sequence).
  void publish(std::size_t count, std::size_t size) {
    for (std::size_t i = 0; i < count; ++i) {
      server.updateMapIncrement(runOf(size, next_x));
      next_x += size;
    }
  }

private:
  std::string address_;

public:
  mslam::SlamServer server;
  std::unique_ptr<sensors::SlamService::Stub> stub;
  std::size_t next_x = 0;

private:
  static std::string address() {
    static std::atomic<int> counter{0};
    return "unix:/tmp/mslam_test_" + std::to_string(::getpid()) + "_" +
           std::to_string(counter++) + ".sock";
  }
};

// One client stream. A deadline keeps a bug from hanging the test.
class MapStream {
public:
  MapStream(RunningServer &s, uint64_t epoch = 0, uint64_t start = 0,
            uint32_t max_points = 0) {
    context.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::seconds(30));
    sensors::MapStreamRequest request;
    request.set_epoch(epoch);
    request.set_start_offset(start);
    request.set_max_points_per_chunk(max_points);
    reader = s.stub->StreamMap(&context, request);
  }
  ~MapStream() {
    context.TryCancel();
    finish();
  }

  // Finish() may be called once only.
  grpc::Status finish() {
    if (!finished) {
      status = reader->Finish();
      finished = true;
    }
    return status;
  }

  bool next(Chunk &chunk) { return reader->Read(&chunk); }

  // Reads chunks until one reports next_offset >= target.
  std::vector<Chunk> readUntil(uint64_t target) {
    std::vector<Chunk> chunks;
    Chunk chunk;
    while (next(chunk)) {
      chunks.push_back(chunk);
      if (chunk.next_offset() >= target && chunk.epoch() != 0 &&
          chunks.size() > 0) {
        break;
      }
    }
    return chunks;
  }

  grpc::ClientContext context;
  std::unique_ptr<grpc::ClientReader<Chunk>> reader;
  grpc::Status status;
  bool finished = false;
};

// The chunks must tile the offsets with no gap or duplicate and carry the x
// sequence 0, 1, 2, ... in order.
void expectSequence(const std::vector<Chunk> &chunks, uint64_t first_offset,
                    uint64_t last_offset, std::size_t first_x,
                    std::size_t total_points) {
  uint64_t offset = first_offset;
  std::size_t x = first_x;
  std::size_t points = 0;
  for (const auto &chunk : chunks) {
    EXPECT_EQ(chunk.first_offset(), offset);
    EXPECT_GE(chunk.next_offset(), chunk.first_offset());
    offset = chunk.next_offset();
    for (int i = 0; i < chunk.points().x_size(); ++i, ++x, ++points) {
      ASSERT_EQ(chunk.points().x(i), static_cast<float>(x));
    }
  }
  EXPECT_EQ(offset, last_offset);
  EXPECT_EQ(points, total_points);
}

TEST(SlamServerStreamMap, FirstMessageIsTheEmptyEpochChunk) {
  RunningServer rs;
  {
    MapStream stream(rs);
    Chunk chunk;
    ASSERT_TRUE(stream.next(chunk));
    EXPECT_NE(chunk.epoch(), 0U);
    EXPECT_EQ(chunk.first_offset(), 0U);
    EXPECT_EQ(chunk.next_offset(), 0U);
    EXPECT_EQ(chunk.points().x_size(), 0);
  }
  // Also when data is waiting: the first message is still the empty chunk.
  rs.publish(5, 10);
  MapStream stream(rs);
  Chunk chunk;
  ASSERT_TRUE(stream.next(chunk));
  EXPECT_EQ(chunk.first_offset(), 0U);
  EXPECT_EQ(chunk.next_offset(), 0U);
  EXPECT_EQ(chunk.points().x_size(), 0);
}

TEST(SlamServerStreamMap, StartFromZeroGetsEverythingInOrder) {
  RunningServer rs;
  rs.publish(40, 7);
  MapStream stream(rs);
  auto chunks = stream.readUntil(40);
  ASSERT_FALSE(chunks.empty());
  EXPECT_EQ(chunks.back().next_offset(), 40U);
  expectSequence(chunks, 0, 40, 0, 40 * 7);
}

TEST(SlamServerStreamMap, LiveClientGetsEveryIncrementEvenInBursts) {
  RunningServer rs;
  MapStream stream(rs);
  Chunk first;
  ASSERT_TRUE(stream.next(first));
  // A small chunk size and a pause per read make several increments pile up
  // between two reads, the case the old stream loses.
  constexpr std::size_t n = 300;
  std::thread publisher([&] {
    for (std::size_t i = 0; i < n; ++i) {
      rs.publish(1, 3);
      if (i % 50 == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  });
  std::vector<Chunk> chunks{first};
  Chunk chunk;
  while (chunks.back().next_offset() < n && stream.next(chunk)) {
    chunks.push_back(chunk);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  publisher.join();
  expectSequence(chunks, 0, n, 0, n * 3);
}

TEST(SlamServerStreamMap, TwoClientsAtDifferentSpeedsGetTheFullSequence) {
  RunningServer rs;
  MapStream fast(rs, 0, 0, 10);
  MapStream slow(rs, 0, 0, 10);
  constexpr std::size_t n = 200;
  std::thread publisher([&] {
    for (std::size_t i = 0; i < n; ++i) {
      rs.publish(1, 10);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });
  std::vector<Chunk> fast_chunks, slow_chunks;
  std::thread fast_reader([&] { fast_chunks = fast.readUntil(n); });
  std::thread slow_reader([&] {
    Chunk chunk;
    while (slow.next(chunk)) {
      slow_chunks.push_back(chunk);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
      if (chunk.next_offset() >= n) {
        break;
      }
    }
  });
  publisher.join();
  fast_reader.join();
  slow_reader.join();
  expectSequence(fast_chunks, 0, n, 0, n * 10);
  expectSequence(slow_chunks, 0, n, 0, n * 10);
}

TEST(SlamServerStreamMap, BatchingRespectsTheRequestedBound) {
  RunningServer rs;
  rs.publish(10, 100);
  MapStream stream(rs, 0, 0, 250);
  auto chunks = stream.readUntil(10);
  expectSequence(chunks, 0, 10, 0, 1000);
  // First message is the empty one, then 2 increments (200 points) a chunk.
  ASSERT_EQ(chunks.size(), 6U);
  for (std::size_t i = 1; i < chunks.size(); ++i) {
    EXPECT_EQ(chunks[i].points().x_size(), 200);
  }
}

TEST(SlamServerStreamMap, DefaultBoundIs65536Points) {
  RunningServer rs;
  rs.publish(5, 30000);
  MapStream stream(rs);
  auto chunks = stream.readUntil(5);
  expectSequence(chunks, 0, 5, 0, 150000);
  // 30000 + 30000 fits, a third increment would exceed 65536.
  ASSERT_EQ(chunks.size(), 4U);
  EXPECT_EQ(chunks[1].points().x_size(), 60000);
  EXPECT_EQ(chunks[2].points().x_size(), 60000);
  EXPECT_EQ(chunks[3].points().x_size(), 30000);
}

TEST(SlamServerStreamMap, RequestsAbove200000AreLowered) {
  RunningServer rs;
  rs.publish(4, 90000);
  MapStream stream(rs, 0, 0, 5000000);
  auto chunks = stream.readUntil(4);
  expectSequence(chunks, 0, 4, 0, 360000);
  // Two increments (180000 points) fit under 200000, three do not.
  ASSERT_EQ(chunks.size(), 3U);
  EXPECT_EQ(chunks[1].points().x_size(), 180000);
  EXPECT_EQ(chunks[2].points().x_size(), 180000);
}

TEST(SlamServerStreamMap, OversizedIncrementGoesAlone) {
  RunningServer rs;
  rs.publish(1, 10);
  rs.publish(1, 500);
  rs.publish(1, 10);
  MapStream stream(rs, 0, 0, 100);
  auto chunks = stream.readUntil(3);
  expectSequence(chunks, 0, 3, 0, 520);
  ASSERT_EQ(chunks.size(), 4U);
  EXPECT_EQ(chunks[1].points().x_size(), 10);
  EXPECT_EQ(chunks[2].points().x_size(), 500);
  EXPECT_EQ(chunks[3].points().x_size(), 10);
}

TEST(SlamServerStreamMap, EmptyIncrementsAdvanceOffsetsWithoutAMessage) {
  RunningServer rs;
  // Published before the client connects: covered by the chunk's range.
  rs.server.updateMapIncrement({});
  rs.publish(1, 5);
  rs.server.updateMapIncrement({});
  rs.publish(1, 5);
  {
    MapStream stream(rs);
    auto chunks = stream.readUntil(4);
    ASSERT_EQ(chunks.size(), 2U);
    EXPECT_EQ(chunks[1].first_offset(), 0U);
    EXPECT_EQ(chunks[1].next_offset(), 4U);
    EXPECT_EQ(chunks[1].points().x_size(), 10);
  }
  // Published while connected: no message of their own, the next chunk starts
  // after them.
  MapStream stream(rs, 0, 0, 0);
  Chunk chunk;
  ASSERT_TRUE(stream.next(chunk));
  auto drained = stream.readUntil(4);
  rs.server.updateMapIncrement({});
  rs.server.updateMapIncrement({});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  rs.publish(1, 4);
  ASSERT_TRUE(stream.next(chunk));
  EXPECT_EQ(chunk.points().x_size(), 4);
  EXPECT_EQ(chunk.first_offset(), 6U);
  EXPECT_EQ(chunk.next_offset(), 7U);
}

TEST(SlamServerStreamMap, ResumeContinuesWithoutDuplicates) {
  RunningServer rs;
  rs.publish(10, 5);
  uint64_t epoch = 0;
  std::vector<Chunk> before;
  {
    MapStream stream(rs, 0, 0, 5); // one increment per chunk
    before = stream.readUntil(4);
    epoch = before.front().epoch();
  }
  ASSERT_EQ(before.back().next_offset(), 4U);
  expectSequence(before, 0, 4, 0, 20);

  MapStream resumed(rs, epoch, 4, 5);
  auto after = resumed.readUntil(10);
  ASSERT_FALSE(after.empty());
  EXPECT_EQ(after.front().epoch(), epoch);
  EXPECT_EQ(after.front().first_offset(), 4U);
  EXPECT_EQ(after.front().next_offset(), 4U);
  EXPECT_EQ(after.front().points().x_size(), 0);
  expectSequence(after, 4, 10, 20, 30);
}

TEST(SlamServerStreamMap, WrongOrZeroEpochStartsAtZero) {
  RunningServer rs;
  rs.publish(6, 4);
  uint64_t epoch = 0;
  {
    MapStream probe(rs);
    Chunk chunk;
    ASSERT_TRUE(probe.next(chunk));
    epoch = chunk.epoch();
  }
  for (uint64_t wrong : {uint64_t{0}, epoch + 1, epoch ^ 0xff}) {
    MapStream stream(rs, wrong, 3);
    auto chunks = stream.readUntil(6);
    ASSERT_FALSE(chunks.empty());
    EXPECT_EQ(chunks.front().epoch(), epoch);
    EXPECT_EQ(chunks.front().first_offset(), 0U);
    EXPECT_EQ(chunks.front().next_offset(), 0U);
    expectSequence(chunks, 0, 6, 0, 24);
  }
}

TEST(SlamServerStreamMap, OffsetPastTheEndIsClamped) {
  RunningServer rs;
  rs.publish(10, 3);
  uint64_t epoch = 0;
  {
    MapStream probe(rs);
    Chunk chunk;
    ASSERT_TRUE(probe.next(chunk));
    epoch = chunk.epoch();
  }
  MapStream stream(rs, epoch, 1000);
  Chunk chunk;
  ASSERT_TRUE(stream.next(chunk));
  EXPECT_EQ(chunk.first_offset(), 10U);
  EXPECT_EQ(chunk.next_offset(), 10U);
  rs.publish(1, 2);
  ASSERT_TRUE(stream.next(chunk));
  EXPECT_EQ(chunk.first_offset(), 10U);
  EXPECT_EQ(chunk.next_offset(), 11U);
  EXPECT_EQ(chunk.points().x_size(), 2);
}

TEST(SlamServerStreamMap, ResetIsAnnouncedAtOnceAndOffsetsRestart) {
  RunningServer rs;
  MapStream stream(rs);
  Chunk first;
  ASSERT_TRUE(stream.next(first));
  rs.publish(3, 4);
  auto chunks = stream.readUntil(3);
  expectSequence(chunks, 0, 3, 0, 12);

  rs.server.reset();
  Chunk announce;
  ASSERT_TRUE(stream.next(announce)); // arrives without a new increment
  EXPECT_NE(announce.epoch(), first.epoch());
  EXPECT_NE(announce.epoch(), 0U);
  EXPECT_EQ(announce.first_offset(), 0U);
  EXPECT_EQ(announce.next_offset(), 0U);
  EXPECT_EQ(announce.points().x_size(), 0);

  rs.next_x = 100;
  rs.publish(2, 4);
  auto after = stream.readUntil(2);
  for (const auto &chunk : after) {
    EXPECT_EQ(chunk.epoch(), announce.epoch());
  }
  expectSequence(after, 0, 2, 100, 8);
}

TEST(SlamServerStreamMap, ResumeWithTheOldEpochAfterAResetStartsAtZero) {
  RunningServer rs;
  rs.publish(5, 2);
  uint64_t old_epoch = 0;
  {
    MapStream probe(rs);
    Chunk chunk;
    ASSERT_TRUE(probe.next(chunk));
    old_epoch = chunk.epoch();
  }
  rs.server.reset();
  rs.next_x = 0;
  rs.publish(2, 2);
  MapStream stream(rs, old_epoch, 5);
  auto chunks = stream.readUntil(2);
  EXPECT_NE(chunks.front().epoch(), old_epoch);
  EXPECT_EQ(chunks.front().next_offset(), 0U);
  expectSequence(chunks, 0, 2, 0, 4);
}

TEST(SlamServerStreamMap, EpochsDifferBetweenServerInstances) {
  uint64_t epochs[2] = {0, 0};
  for (auto &epoch : epochs) {
    RunningServer rs;
    MapStream stream(rs);
    Chunk chunk;
    ASSERT_TRUE(stream.next(chunk));
    epoch = chunk.epoch();
  }
  EXPECT_NE(epochs[0], 0U);
  EXPECT_NE(epochs[1], 0U);
  EXPECT_NE(epochs[0], epochs[1]);
}

TEST(SlamServerStreamMap, StreamEndsWhenTheServerStops) {
  RunningServer rs;
  MapStream stream(rs);
  Chunk chunk;
  ASSERT_TRUE(stream.next(chunk));
  const auto begin = Clock::now();
  std::thread stopper([&] { rs.server.stop(); });
  while (stream.next(chunk)) {
  }
  stopper.join();
  EXPECT_LT(Clock::now() - begin, std::chrono::seconds(3));
  EXPECT_TRUE(stream.finish().ok());
}

TEST(SlamServerStreamMap, StreamEndsOnClientCancelAndStopIsPrompt) {
  RunningServer rs;
  MapStream stream(rs);
  Chunk chunk;
  ASSERT_TRUE(stream.next(chunk));
  stream.context.TryCancel();
  EXPECT_FALSE(stream.next(chunk));
  // Give the handler a poll period to notice, then stop() must not wait on it.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  const auto begin = Clock::now();
  rs.server.stop();
  EXPECT_LT(Clock::now() - begin, std::chrono::seconds(2));
}

TEST(SlamServerStreamMap, ABlockedClientDoesNotBlockThePublisherOrOthers) {
  RunningServer rs;
  MapStream stalled(rs, 0, 0, 0);
  Chunk first;
  ASSERT_TRUE(stalled.next(first));
  // The stalled client never reads again: its Write blocks once the transport
  // window is full while a large backlog builds up behind it.
  MapStream healthy(rs);
  Chunk chunk;
  ASSERT_TRUE(healthy.next(chunk));

  constexpr std::size_t n = 200;
  std::chrono::nanoseconds worst{0};
  std::vector<Chunk> healthy_chunks;
  std::thread reader([&] { healthy_chunks = healthy.readUntil(n); });
  for (std::size_t i = 0; i < n; ++i) {
    const auto begin = Clock::now();
    rs.publish(1, 20000);
    worst =
        std::max(worst, std::chrono::duration_cast<std::chrono::nanoseconds>(
                            Clock::now() - begin));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  reader.join();
  // Building a 20000-point increment is included; a lock held across a
  // blocked Write would stall for seconds.
  EXPECT_LT(worst, std::chrono::milliseconds(250));
  expectSequence(healthy_chunks, 0, n, 0, n * 20000);
}

// ---------------------------------------------------------------------------
// Sensor link status.

using namespace std::chrono_literals;

TEST(SensorLinkClassification, NoLinkIsDisconnectedWhateverTheAges) {
  EXPECT_EQ(mslam::classifySensorLink(false, std::nullopt, std::nullopt),
            mslam::SensorLink::Disconnected);
  EXPECT_EQ(mslam::classifySensorLink(false, 10ms, 10ms),
            mslam::SensorLink::Disconnected);
}

TEST(SensorLinkClassification, ConnectedWithoutSamplesIsIdle) {
  // A paused playback server: connected, nothing arrives.
  EXPECT_EQ(mslam::classifySensorLink(true, std::nullopt, std::nullopt),
            mslam::SensorLink::ConnectedIdle);
}

TEST(SensorLinkClassification, StreamingNeedsBothStreamsFresh) {
  EXPECT_EQ(mslam::classifySensorLink(true, 48ms, 5ms),
            mslam::SensorLink::Streaming);
  // The limit itself still counts as arriving.
  EXPECT_EQ(mslam::classifySensorLink(true, mslam::kSensorStreamingMaxAge, 0ms),
            mslam::SensorLink::Streaming);
  EXPECT_EQ(mslam::classifySensorLink(true, 48ms, std::nullopt),
            mslam::SensorLink::ConnectedIdle);
  EXPECT_EQ(mslam::classifySensorLink(true, std::nullopt, 5ms),
            mslam::SensorLink::ConnectedIdle);
  EXPECT_EQ(
      mslam::classifySensorLink(true, mslam::kSensorStreamingMaxAge + 1ms, 5ms),
      mslam::SensorLink::ConnectedIdle);
  EXPECT_EQ(mslam::classifySensorLink(true, 48ms, 10s),
            mslam::SensorLink::ConnectedIdle);
}

TEST_F(SlamServerTest, StatusIsUnknownWithoutASensorStatusSource) {
  sensors::Empty request;
  sensors::SlamStatus response;
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.sensor_link(), sensors::SENSOR_LINK_UNKNOWN);
  EXPECT_TRUE(response.sensor_address().empty());
  EXPECT_EQ(response.scan_age_ms(), -1);
  EXPECT_EQ(response.imu_age_ms(), -1);
}

TEST_F(SlamServerTest, StatusReportsTheSensorLink) {
  sensors::Empty request;
  sensors::SlamStatus response;
  auto source = std::make_shared<FakeSensorStatus>();
  server_.setSensorStatus(source);

  source->status = {.link = mslam::SensorLink::Streaming,
                    .address = "10.0.0.2:50051",
                    .scan_age = 48ms,
                    .imu_age = 5ms};
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.sensor_link(), sensors::SENSOR_LINK_STREAMING);
  EXPECT_EQ(response.sensor_address(), "10.0.0.2:50051");
  EXPECT_EQ(response.scan_age_ms(), 48);
  EXPECT_EQ(response.imu_age_ms(), 5);

  source->status = {.link = mslam::SensorLink::ConnectedIdle,
                    .address = "10.0.0.2:50051",
                    .scan_age = 90'000ms};
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.sensor_link(), sensors::SENSOR_LINK_CONNECTED_IDLE);
  EXPECT_EQ(response.scan_age_ms(), 90'000);
  EXPECT_EQ(response.imu_age_ms(), -1);

  source->status = {.link = mslam::SensorLink::Disconnected};
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.sensor_link(), sensors::SENSOR_LINK_DISCONNECTED);

  source->status = {.link = mslam::SensorLink::NotApplicable,
                    .address = "local"};
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.sensor_link(), sensors::SENSOR_LINK_NOT_APPLICABLE);
  EXPECT_EQ(response.sensor_address(), "local");

  // An age beyond int32 saturates instead of wrapping.
  source->status = {.link = mslam::SensorLink::ConnectedIdle,
                    .scan_age = std::chrono::hours(24 * 60)};
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.scan_age_ms(), std::numeric_limits<int32_t>::max());

  server_.setSensorStatus(nullptr);
  ASSERT_TRUE(service().GetStatus(nullptr, &request, &response).ok());
  EXPECT_EQ(response.sensor_link(), sensors::SENSOR_LINK_UNKNOWN);
}

TEST(SlamServerStatus, IsServedOverTheWire) {
  RunningServer rs;
  const auto get = [&] {
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + 5s);
    sensors::SlamStatus status;
    EXPECT_TRUE(rs.stub->GetStatus(&context, sensors::Empty{}, &status).ok());
    return status;
  };
  EXPECT_EQ(get().sensor_link(), sensors::SENSOR_LINK_UNKNOWN);

  auto source = std::make_shared<FakeSensorStatus>();
  source->status = {.link = mslam::SensorLink::Streaming,
                    .address = "sensor:50051",
                    .scan_age = 10ms,
                    .imu_age = 2ms};
  rs.server.setSensorStatus(source);
  auto status = get();
  EXPECT_EQ(status.sensor_link(), sensors::SENSOR_LINK_STREAMING);
  EXPECT_EQ(status.sensor_address(), "sensor:50051");

  source->status = {.link = mslam::SensorLink::Disconnected,
                    .address = "sensor:50051"};
  EXPECT_EQ(get().sensor_link(), sensors::SENSOR_LINK_DISCONNECTED);
}

} // namespace
