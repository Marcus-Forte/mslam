#include "mslam/config/Validation.hh"
#include <regex>
#include <stdexcept>
#include <string>

static bool isValidIPv4(const std::string &ip) {
  std::regex ipPattern(R"((\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3}))");
  std::smatch match;

  // MacOS host := host.docker.internal
  if (ip == "localhost" || ip == "host.docker.internal" ||
      ip == "docker.host.internal") {
    return true;
  }

  if (!std::regex_match(ip, match, ipPattern)) {
    return false;
  }

  for (int i = 1; i <= 4; ++i) {
    int num = std::stoi(match[i].str());
    if (num < 0 || num > 255)
      return false;
  }

  return true;
}

static bool isValidPort(const std::string &portStr) {
  if (portStr.empty() || portStr.length() > 5)
    return false;

  for (char ch : portStr) {
    if (!isdigit(ch))
      return false;
  }

  int port = std::stoi(portStr);
  return (port >= 0 && port <= 65535);
}

namespace mslam {
bool isValidIPAndPort(const std::string &input) {
  if (input == "local") {
    return true;
  }
  size_t colonPos = input.rfind(':');
  if (colonPos == std::string::npos)
    return false; // No colon found

  std::string ip = input.substr(0, colonPos);
  std::string port = input.substr(colonPos + 1);

  return isValidIPv4(ip) && isValidPort(port);
}
} // namespace mslam
namespace mslam {
void rejectUnknownKeys(const Json::Value &object, std::string_view section,
                       std::initializer_list<std::string_view> known,
                       std::string_view what) {
  for (const auto &name : object.getMemberNames()) {
    bool found = false;
    for (const auto &candidate : known) {
      found = found || candidate == name;
    }
    if (!found) {
      throw std::runtime_error(
          std::string(what) + ": unknown key '" +
          (section.empty() ? name : std::string(section) + "." + name) + "'");
    }
  }
}
} // namespace mslam

namespace mslam {
SyclSettings parseSyclSettings(const Json::Value &block,
                               std::string_view what) {
  SyclSettings settings;
  if (block.isNull()) {
    return settings;
  }
  if (!block.isObject()) {
    throw std::runtime_error(std::string(what) +
                             ": 'sycl' must be an object");
  }
  rejectUnknownKeys(block, "sycl", {"enabled", "threads"}, what);
  if (block.isMember("enabled")) {
    if (!block["enabled"].isBool()) {
      throw std::runtime_error(std::string(what) +
                               ": 'sycl.enabled' must be true or false");
    }
    settings.enabled = block["enabled"].asBool();
  }
  if (block.isMember("threads")) {
    constexpr double kMaxThreads = 256; // only to reject nonsense
    const auto &threads = block["threads"];
    // Only a JSON integer token is a count (jsoncpp's isIntegral accepts 2.0).
    const bool is_integer_token =
        threads.type() == Json::intValue || threads.type() == Json::uintValue;
    if (!is_integer_token || threads.asDouble() < 0.0 ||
        threads.asDouble() > kMaxThreads) {
      throw std::runtime_error(std::string(what) +
                               ": 'sycl.threads' must be an integer in [0, 256]");
    }
    settings.threads = threads.asUInt();
  }
  return settings;
}
} // namespace mslam
