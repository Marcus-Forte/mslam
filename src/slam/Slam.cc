#include "slam/Slam.hh"

namespace mslam {

Slam::Slam(const SlamParameters &config) : config_(config) {}
void Slam::Predict(const msensor::IMUData &imuData) {}
void Slam::Update(const msensor::Scan3DI &lidarData) {}
} // namespace mslam