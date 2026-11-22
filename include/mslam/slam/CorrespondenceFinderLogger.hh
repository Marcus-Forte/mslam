#pragma once

#include "mslam/slam/CorrespondenceFinder.hh"
#include <memory>
#include <spdlog/logger.h>

namespace mslam {

/// A CorrespondenceFinder that logs each search at debug level.
std::shared_ptr<CorrespondenceFinder> createLoggingCorrespondenceFinder(
    const std::shared_ptr<spdlog::logger> &logger);

} // namespace mslam
