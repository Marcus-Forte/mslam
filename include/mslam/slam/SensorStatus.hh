#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace mslam {

/// State of the link to the sensor server, as reported to viewers.
enum class SensorLink {
  Unknown,       ///< No status source attached.
  NotApplicable, ///< The sensor is on-board; there is no link.
  Disconnected,  ///< The sensor server is unreachable.
  ConnectedIdle, ///< Connected, but samples are not arriving on both streams.
  Streaming,     ///< Connected and LiDAR and IMU samples are arriving.
};

/// A stream counts as arriving while its last sample is at most this old. A
/// Mid360 scan takes 48 ms, so this is generous; playback at 1/8 speed is
/// still well inside it.
constexpr std::chrono::milliseconds kSensorStreamingMaxAge{2000};

/// Classifies the link from the connection state and the age of the last
/// sample of each stream (empty = none yet). Streaming needs both streams
/// fresh because the SLAM needs both. A connected server that stays silent
/// (a paused playback) is ConnectedIdle, not Disconnected.
inline SensorLink
classifySensorLink(bool link_up,
                   std::optional<std::chrono::milliseconds> scan_age,
                   std::optional<std::chrono::milliseconds> imu_age,
                   std::chrono::milliseconds max_age = kSensorStreamingMaxAge) {
  if (!link_up) {
    return SensorLink::Disconnected;
  }
  const auto fresh = [max_age](std::optional<std::chrono::milliseconds> age) {
    return age.has_value() && *age <= max_age;
  };
  return fresh(scan_age) && fresh(imu_age) ? SensorLink::Streaming
                                           : SensorLink::ConnectedIdle;
}

struct SensorStatus {
  SensorLink link = SensorLink::Unknown;
  std::string address;
  std::optional<std::chrono::milliseconds> scan_age;
  std::optional<std::chrono::milliseconds> imu_age;
};

/// Read-only view of the sensor link, callable from any thread (the gRPC
/// server's handlers).
class ISensorStatus {
public:
  virtual ~ISensorStatus() = default;
  virtual SensorStatus sensorStatus() const = 0;
};

} // namespace mslam
