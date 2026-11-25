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
} // namespace mslam
