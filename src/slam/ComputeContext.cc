#include "mslam/slam/ComputeContext.hh"

#include "moptim/ExecutionContext.hh"

#include <mutex>
#include <stdexcept>

namespace mslam {

namespace {
std::mutex g_mutex;
std::shared_ptr<moptim::ExecutionContext> g_host;
std::shared_ptr<moptim::ExecutionContext> g_sycl;
} // namespace

bool syclBackendAvailable() { return moptim::syclAvailable(); }

std::shared_ptr<moptim::ExecutionContext>
executionContextFor(ComputeBackend backend) {
  const std::lock_guard lock(g_mutex);
  if (backend == ComputeBackend::Host) {
    if (!g_host) {
      g_host =
          std::make_shared<moptim::ExecutionContext>(moptim::Backend::Host);
    }
    return g_host;
  }
  if (!moptim::syclAvailable()) {
    throw std::runtime_error(
        "sycl.enabled is true, but this mslam was built without SYCL "
        "(configure with -DMSLAM_WITH_SYCL=ON, e.g. the sycl preset)");
  }
  if (!g_sycl) {
    g_sycl = std::make_shared<moptim::ExecutionContext>(moptim::Backend::Sycl);
  }
  return g_sycl;
}

std::size_t deviceWidth(ComputeBackend backend, unsigned sycl_threads) {
  if (backend == ComputeBackend::Host) {
    return 1;
  }
  if (sycl_threads != 0) {
    return sycl_threads;
  }
  // launchWidth() is min(compute units, partitions): ask for more partitions
  // than any device has units.
  return executionContextFor(backend)->launchWidth(std::size_t{1} << 20);
}

} // namespace mslam
