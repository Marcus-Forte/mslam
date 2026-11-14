#include "mslam/utils/Timing.hh"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <sstream>

namespace {

std::shared_ptr<spdlog::logger> makeLogger(std::ostringstream &stream,
                                           spdlog::level::level_enum level) {
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(stream);
  auto logger = std::make_shared<spdlog::logger>("timing_test", sink);
  logger->set_level(level);
  return logger;
}

TEST(ScopedElapsedLogger, LogsTheScopeNameAtTheRequestedLevel) {
  std::ostringstream out;
  const auto logger = makeLogger(out, spdlog::level::info);
  {
    const mslam::ScopedElapsedLogger timer(logger, "Stage");
  }
  EXPECT_NE(out.str().find("Stage elapsed:"), std::string::npos);
  EXPECT_NE(out.str().find("[info]"), std::string::npos);
}

TEST(ScopedElapsedLogger, StaysSilentBelowTheLoggerLevel) {
  std::ostringstream out;
  const auto logger = makeLogger(out, spdlog::level::info);
  {
    const mslam::ScopedElapsedLogger timer(logger, "Hot", spdlog::level::debug);
  }
  EXPECT_TRUE(out.str().empty());
}

TEST(ScopedElapsedLogger, LogsDebugScopesWhenDebugIsEnabled) {
  std::ostringstream out;
  const auto logger = makeLogger(out, spdlog::level::debug);
  {
    const mslam::ScopedElapsedLogger timer(logger, "Hot", spdlog::level::debug);
  }
  EXPECT_NE(out.str().find("Hot elapsed:"), std::string::npos);
  EXPECT_NE(out.str().find("[debug]"), std::string::npos);
}

TEST(ScopedElapsedLogger, ToleratesANullLogger) {
  const std::shared_ptr<spdlog::logger> none;
  {
    const mslam::ScopedElapsedLogger timer(none, "Nothing");
  }
  SUCCEED();
}

} // namespace
