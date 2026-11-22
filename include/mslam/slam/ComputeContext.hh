#pragma once

#include "mslam/config/IConfig.hh"

#include <cstddef>
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

/// Work-groups of mslam's own SYCL kernels (the correspondence search, the
/// preprocessing): @p sycl_threads (slam.sycl.threads) when not 0, otherwise
/// every compute unit of the SYCL device. 1 on the host backend. Results do not
/// depend on it.
std::size_t deviceWidth(ComputeBackend backend, unsigned sycl_threads);

/// Whether this build has the SYCL backend (MSLAM_WITH_SYCL).
bool syclBackendAvailable();

} // namespace mslam
