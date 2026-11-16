#include "mslam/utils/Parallel.hh"

#ifdef MSLAM_HAS_TBB
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/info.h>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/partitioner.h>
#include <oneapi/tbb/task_arena.h>
#endif

#include <algorithm>

namespace mslam {

#ifdef MSLAM_HAS_TBB

struct ParallelExecutor::Impl {
  explicit Impl(int max_concurrency) : arena(max_concurrency) {
    // Create the arena now, so the first scan does not pay for it.
    arena.initialize();
  }
  // execute() is not const; concurrent callers are allowed by TBB.
  mutable tbb::task_arena arena;
};

ParallelExecutor::ParallelExecutor(unsigned num_threads) {
  const auto hardware =
      static_cast<unsigned>(std::max(1, tbb::info::default_concurrency()));
  const unsigned wanted = num_threads == 0 ? hardware : num_threads;
  if (wanted > 1) {
    impl_ = std::make_unique<Impl>(static_cast<int>(wanted));
    concurrency_ = static_cast<unsigned>(impl_->arena.max_concurrency());
  }
}

bool ParallelExecutor::available() noexcept { return true; }

void ParallelExecutor::run(std::size_t n, std::size_t grain,
                           RangeFunction function, void *context) const {
  impl_->arena.execute([&] {
    // static_partitioner: one contiguous block per thread (none smaller than
    // the grain), no further splitting or stealing. The regions take a
    // fraction of a millisecond, so few tasks beat finer balancing: it was
    // faster than auto_partitioner and simple_partitioner on
    // living_room_garden at 4 threads (sweep in improvement_tbb.md).
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, n, grain),
        [&](const tbb::blocked_range<std::size_t> &range) {
          function(context, range.begin(), range.end());
        },
        tbb::static_partitioner{});
  });
}

#else

struct ParallelExecutor::Impl {};

ParallelExecutor::ParallelExecutor(unsigned /*num_threads*/) {}

bool ParallelExecutor::available() noexcept { return false; }

void ParallelExecutor::run(std::size_t n, std::size_t /*grain*/,
                           RangeFunction function, void *context) const {
  function(context, 0, n);
}

#endif

ParallelExecutor::~ParallelExecutor() = default;

std::shared_ptr<const ParallelExecutor>
makeParallelExecutor(unsigned num_threads, spdlog::logger &logger) {
  if (num_threads == 1) {
    return nullptr;
  }
  if (!ParallelExecutor::available()) {
    logger.warn("num_threads is {}, but mslam was built without TBB; running "
                "sequentially",
                num_threads);
    return nullptr;
  }
  return std::make_shared<const ParallelExecutor>(num_threads);
}

} // namespace mslam
