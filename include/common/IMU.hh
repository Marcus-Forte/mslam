#pragma once
#include <stdint.h>

namespace mslam {
struct IMUData {
  uint64_t timestamp;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
} // namespace mslam