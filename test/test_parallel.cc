#include "mslam/utils/Parallel.hh"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <cstddef>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

namespace {

using mslam::ParallelExecutor;

constexpr std::size_t kGrain = 16;

// Counts the visits of every index of [0, n).
std::vector<int> visitCounts(const ParallelExecutor &executor, std::size_t n,
                             std::size_t grain) {
  std::vector<std::atomic<int>> counts(n);
  executor.forRange(n, grain, [&](std::size_t begin, std::size_t end) {
    ASSERT_LE(begin, end);
    ASSERT_LE(end, n);
    for (std::size_t i = begin; i < end; ++i) {
      counts[i].fetch_add(1, std::memory_order_relaxed);
    }
  });
  std::vector<int> result;
  result.reserve(n);
  for (const auto &count : counts) {
    result.push_back(count.load());
  }
  return result;
}

class ParallelForRange : public ::testing::TestWithParam<unsigned> {};

TEST_P(ParallelForRange, VisitsEveryIndexExactlyOnce) {
  const ParallelExecutor executor(GetParam());
  for (const std::size_t n :
       {std::size_t{0}, std::size_t{1}, kGrain - 1, kGrain, 2 * kGrain - 1,
        2 * kGrain, 2 * kGrain + 1, 10 * kGrain + 3, std::size_t{10007}}) {
    const auto counts = visitCounts(executor, n, kGrain);
    ASSERT_EQ(counts.size(), n);
    for (std::size_t i = 0; i < n; ++i) {
      ASSERT_EQ(counts[i], 1) << "n=" << n << " index " << i;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Threads, ParallelForRange,
                         ::testing::Values(1U, 2U, 4U, 0U));

TEST(ParallelExecutor, OneThreadRunsTheWholeRangeOnTheCaller) {
  const ParallelExecutor executor(1);
  EXPECT_EQ(executor.concurrency(), 1U);
  int calls = 0;
  std::thread::id thread;
  executor.forRange(10007, kGrain, [&](std::size_t begin, std::size_t end) {
    ++calls;
    thread = std::this_thread::get_id();
    EXPECT_EQ(begin, 0U);
    EXPECT_EQ(end, 10007U);
  });
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(thread, std::this_thread::get_id());
}

TEST(ParallelExecutor, SmallRangesRunInlineOnTheCaller) {
  const ParallelExecutor executor(4);
  int calls = 0;
  std::thread::id thread;
  executor.forRange(2 * kGrain - 1, kGrain,
                    [&](std::size_t begin, std::size_t end) {
                      ++calls;
                      thread = std::this_thread::get_id();
                      EXPECT_EQ(begin, 0U);
                      EXPECT_EQ(end, 2 * kGrain - 1);
                    });
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(thread, std::this_thread::get_id());
}

TEST(ParallelExecutor, ConcurrencyIsBoundedByTheRequest) {
  if (!ParallelExecutor::available()) {
    GTEST_SKIP() << "built without TBB";
  }
  const ParallelExecutor executor(2);
  EXPECT_EQ(executor.concurrency(), 2U);

  // Never more distinct threads than the arena has slots.
  std::mutex mutex;
  std::set<std::thread::id> threads;
  for (int repeat = 0; repeat < 20; ++repeat) {
    executor.forRange(4096, 8, [&](std::size_t, std::size_t) {
      const std::lock_guard lock(mutex);
      threads.insert(std::this_thread::get_id());
    });
  }
  EXPECT_LE(threads.size(), 2U);
  EXPECT_GE(ParallelExecutor(0).concurrency(), 1U);
}

TEST(ParallelExecutor, MakeReturnsNullForOneThread) {
  std::ostringstream log;
  auto logger = std::make_shared<spdlog::logger>(
      "parallel_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(log));
  EXPECT_EQ(mslam::makeParallelExecutor(1, *logger), nullptr);

  const auto executor = mslam::makeParallelExecutor(4, *logger);
  if (ParallelExecutor::available()) {
    ASSERT_NE(executor, nullptr);
    EXPECT_EQ(executor->concurrency(), 4U);
    EXPECT_TRUE(log.str().empty());
  } else {
    EXPECT_EQ(executor, nullptr);
    EXPECT_NE(log.str().find("without TBB"), std::string::npos);
  }
}

} // namespace
