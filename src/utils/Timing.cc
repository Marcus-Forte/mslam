#include "mslam/utils/Timing.hh"

#include <chrono>

namespace mslam {

ScopedElapsedLogger::ScopedElapsedLogger(
    const std::shared_ptr<spdlog::logger> &logger, std::string_view name,
    spdlog::level::level_enum level)
    : logger_(logger && logger->should_log(level) ? logger.get() : nullptr),
      name_(name), level_(level) {
  if (logger_ != nullptr) {
    start_ = std::chrono::steady_clock::now();
  }
}

ScopedElapsedLogger::~ScopedElapsedLogger() {
  if (logger_ == nullptr) {
    return;
  }
  const auto elapsed = std::chrono::steady_clock::now() - start_;
  logger_->log(level_, "{} elapsed: {:.3f} ms", name_,
               std::chrono::duration<double, std::milli>(elapsed).count());
}

} // namespace mslam
