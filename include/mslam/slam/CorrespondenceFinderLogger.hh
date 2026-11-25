#pragma once

#include "mslam/slam/CorrespondenceFinder.hh"
#include <memory>
#include <spdlog/logger.h>

namespace mslam {

/// A CorrespondenceFinder that logs each search at debug level. @p executor
/// is forwarded to it (nullptr: sequential search).
std::shared_ptr<CorrespondenceFinder> createLoggingCorrespondenceFinder(
    const std::shared_ptr<spdlog::logger> &logger,
    std::shared_ptr<const ParallelExecutor> executor = nullptr);

} // namespace mslam
