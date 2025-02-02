#pragma once

#include "ISlam.hh"
#include "config/IConfig.hh"
#include "map/IMap.hh"

namespace mslam {

class Slam : public ISlam {
public:
  Slam(const SlamParameters &config);
  virtual void Predict(const msensor::IMUData &imuData);
  virtual void Update(const msensor::Scan3DI &lidarData);

private:
  SlamParameters config_;
  std::unique_ptr<IMap> map_;
};
} // namespace mslam