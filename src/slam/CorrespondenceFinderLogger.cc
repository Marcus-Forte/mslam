#include "slam/CorrespondenceFinderLogger.hh"
#include <utility>

namespace mslam {
namespace {

class CorrespondenceSearchLogger : public ICorrespondenceFinderObserver {
public:
  explicit CorrespondenceSearchLogger(std::shared_ptr<spdlog::logger> logger)
      : logger_(std::move(logger)) {}

  void onCorrespondenceSearch(const CorrespondenceSearchEvent &event) override {
    logger_->debug("KNN Search. Correspondences: {} / {} ({} us)",
                   event.correspondence_count, event.scan_size,
                   event.elapsed.count() / 1000);
  }

private:
  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace

std::shared_ptr<CorrespondenceFinder> createLoggingCorrespondenceFinder(
    const std::shared_ptr<spdlog::logger> &logger) {
  auto finder = std::make_shared<CorrespondenceFinder>();
  finder->setObserver(std::make_shared<CorrespondenceSearchLogger>(logger));
  return finder;
}

} // namespace mslam
