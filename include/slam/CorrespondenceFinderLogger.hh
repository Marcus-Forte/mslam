#pragma once

#include "slam/CorrespondenceFinder.hh"
#include <memory>
#include <spdlog/logger.h>

namespace mslam {

std::shared_ptr<CorrespondenceFinder> createLoggingCorrespondenceFinder(
    const std::shared_ptr<spdlog::logger> &logger);

} // namespace mslam
