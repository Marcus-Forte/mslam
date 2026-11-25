#pragma once

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#include <spdlog/logger.h>

namespace mslam {

/// Runs index ranges on a bounded set of threads (a oneTBB task arena when
/// mslam is built with TBB, the calling thread otherwise).
///
/// The callers rely on one rule to get results that do not depend on the
/// thread count: a body writes only the output slots of its own indexes, and
/// everything order-dependent (compaction, cache insertion, accumulation) stays
/// outside the body, sequential and in index order.
///
/// forRange() may be called from several threads at once; each call blocks
/// until its range is done. TBB types stay out of this header.
class ParallelExecutor {
public:
  /// @param num_threads 1 runs everything on the caller, 0 uses every hardware
  /// thread, n uses at most n threads, the caller included. Without TBB the
  /// executor is always sequential.
  explicit ParallelExecutor(unsigned num_threads);
  ~ParallelExecutor();

  ParallelExecutor(const ParallelExecutor &) = delete;
  ParallelExecutor &operator=(const ParallelExecutor &) = delete;

  /// Threads a range may run on, the caller included (1 when sequential).
  unsigned concurrency() const noexcept { return concurrency_; }

  /// Whether this build can run anything in parallel (built with TBB).
  static bool available() noexcept;

  /// Calls body(begin, end) on disjoint sub-ranges that cover [0, n), each at
  /// least @p grain long except possibly the last ones. Runs body(0, n) on the
  /// caller when the executor is sequential or n < 2 * grain, so small inputs
  /// pay no task overhead. The body must not throw.
  template <class Body>
  void forRange(std::size_t n, std::size_t grain, Body &&body) const {
    if (n == 0) {
      return;
    }
    if (concurrency_ <= 1 || grain == 0 || n < 2 * grain) {
      body(std::size_t{0}, n);
      return;
    }
    using BodyType = std::remove_reference_t<Body>;
    // Non-owning type erasure: no allocation per parallel region.
    run(
        n, grain,
        [](void *context, std::size_t begin, std::size_t end) {
          (*static_cast<BodyType *>(context))(begin, end);
        },
        const_cast<void *>(static_cast<const void *>(&body)));
  }

private:
  using RangeFunction = void (*)(void *context, std::size_t begin,
                                 std::size_t end);
  void run(std::size_t n, std::size_t grain, RangeFunction function,
           void *context) const;

  struct Impl;
  std::unique_ptr<Impl> impl_;
  unsigned concurrency_ = 1;
};

/// The executor for a `num_threads` setting: nullptr (the sequential code path)
/// for 1, and also, after one warning on @p logger, when the build has no TBB.
std::shared_ptr<const ParallelExecutor>
makeParallelExecutor(unsigned num_threads, spdlog::logger &logger);

} // namespace mslam
