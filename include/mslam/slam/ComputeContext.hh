#pragma once

#include "mslam/config/IConfig.hh"

#include <memory>

namespace moptim {
class ExecutionContext;
}

namespace mslam {

/// The moptim execution context of @p backend: one per process and backend, so
/// the map, the preprocessing and the registrations share it (on SYCL one
/// queue). Throws std::runtime_error for ComputeBackend::Sycl in a build without
/// SYCL (MSLAM_WITH_SYCL).
std::shared_ptr<moptim::ExecutionContext>
executionContextFor(ComputeBackend backend);

/// Whether this build has the SYCL backend (MSLAM_WITH_SYCL).
bool syclBackendAvailable();

} // namespace mslam
