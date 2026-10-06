#pragma once

#include "moptim/Observer.hh"

#include <memory>
#include <spdlog/logger.h>
#include <utility>

namespace mslam {

/**
 * @brief Adapter that forwards moptim telemetry to spdlog.
 *
 * moptim no longer contains a logging framework; it emits structured events
 * through moptim::IOptimizerObserver. Attach an instance with
 * moptim::IOptimizer::setObserver(). The observer is non-owning, so keep it
 * alive for the duration of optimize()/step().
 */
template <class T>
class OptimizerObserver : public moptim::IOptimizerObserver<T> {
public:
  explicit OptimizerObserver(std::shared_ptr<spdlog::logger> logger)
      : logger_(std::move(logger)) {}

  void onIteration(const moptim::IterationEvent<T> &event) override {
    logger_->debug("iter {} trial {} phase {} status {} cost {} -> {} rho {} "
                   "lambda {} delta {} ({} us)",
                   event.iteration, event.trial, static_cast<int>(event.phase),
                   static_cast<int>(event.status), event.cost,
                   event.previous_cost, event.rho, event.lambda,
                   event.delta_norm, event.elapsed.count() / 1000);
  }

  void onLinearSystem(const moptim::LinearSystemEvent<T> &event) override {
    logger_->debug("iter {} linear system cost {} ({} us)", event.iteration,
                   event.cost, event.elapsed.count() / 1000);
  }

private:
  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace mslam
