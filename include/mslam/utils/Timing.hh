#pragma once

#include <chrono>
#include <string_view>

#include <spdlog/logger.h>

namespace mslam {

/// Logs the lifetime of a scope. When the logger does not log at @p level the
/// clock is never read and nothing is formatted, so it is cheap to leave in
/// hot paths.
class ScopedElapsedLogger {
public:
  /// @param name Must outlive this object (use a string literal).
  ScopedElapsedLogger(const std::shared_ptr<spdlog::logger> &logger,
                      std::string_view name,
                      spdlog::level::level_enum level = spdlog::level::info);
  ~ScopedElapsedLogger();

  ScopedElapsedLogger(const ScopedElapsedLogger &) = delete;
  ScopedElapsedLogger &operator=(const ScopedElapsedLogger &) = delete;

private:
  spdlog::logger *logger_; ///< null when the level is disabled
  std::string_view name_;
  spdlog::level::level_enum level_;
  std::chrono::steady_clock::time_point start_;
};

} // namespace mslam
