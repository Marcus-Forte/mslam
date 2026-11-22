#pragma once

#include "jsoncpp/json/value.h"

#include <initializer_list>
#include <string>
#include <string_view>

namespace mslam {
bool isValidIPAndPort(const std::string &input);

/// Throws std::runtime_error("<what>: unknown key '<section>.<key>'") for the
/// first member of `object` that is not in `known`. An empty `section` names
/// the key alone.
void rejectUnknownKeys(const Json::Value &object, std::string_view section,
                       std::initializer_list<std::string_view> known,
                       std::string_view what);

/// The optional top-level "sycl" block (`sycl.enabled`, `sycl.threads`) of the
/// SLAM and registration configs.
struct SyclSettings {
  bool enabled = false;
  unsigned threads = 0; ///< Work-groups of mslam's kernels; 0: every compute unit.
};

/// Reads the "sycl" block, @p block being null (absent) or an object. Throws
/// std::runtime_error("<what>: ...") for an unknown key, a non-boolean
/// `enabled` or a `threads` that is not an integer in [0, 256].
SyclSettings parseSyclSettings(const Json::Value &block, std::string_view what);
} // namespace mslam
