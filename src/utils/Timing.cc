#include "mslam/utils/Timing.hh"

#include <chrono>
#include <utility>

namespace mslam {

ScopedElapsedLogger::ScopedElapsedLogger(std::shared_ptr<spdlog::logger> logger,
                                         std::string name)
    : logger_(std::move(logger)), name_(std::move(name)),
      start_(std::chrono::steady_clock::now()) {}

ScopedElapsedLogger::~ScopedElapsedLogger() {
  const auto elapsed = std::chrono::steady_clock::now() - start_;
  logger_->info("{} elapsed: {:.3f} ms", name_,
                std::chrono::duration<double, std::milli>(elapsed).count());
}

} // namespace mslam