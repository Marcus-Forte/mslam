#pragma once

#include "config/IConfig.hh"
#include <lidar/ILidar.hh>

namespace mslam {
class Preprocessor {
public:
  Preprocessor(const PreProcessor &config);

  /**
   * @brief Downsamples the input
   * \note the input will be moved to the output!
   * @param input
   * @return std::shared_ptr<msensor::Scan3D>
   */
  std::shared_ptr<msensor::Scan3D> downsample(msensor::Scan3D &input);

private:
  PreProcessor config_;
};
} // namespace mslam