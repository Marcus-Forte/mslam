#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <spdlog/logger.h>

namespace mslam {

class ScopedElapsedLogger {
public:
  ScopedElapsedLogger(std::shared_ptr<spdlog::logger> logger, std::string name);
  ~ScopedElapsedLogger();

  ScopedElapsedLogger(const ScopedElapsedLogger &) = delete;
  ScopedElapsedLogger &operator=(const ScopedElapsedLogger &) = delete;

private:
  std::shared_ptr<spdlog::logger> logger_;
  std::string name_;
  std::chrono::steady_clock::time_point start_;
};

} // namespace mslam
