#include "config/Validation.hh"
#include <regex>

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