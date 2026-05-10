#pragma once

#include "config/IConfig.hh"
#include "interface/ILidar.hh"

namespace mslam {
class Preprocessor {
public:
  Preprocessor(const PreProcessor &config);

  /**
   * @brief Removes near-center points and downsamples the input scan.
   */
  std::shared_ptr<msensor::Scan3D>
  preprocess(const msensor::Scan3D &input) const;

  /**
   * @brief Downsamples the input scan.
   */
  std::shared_ptr<msensor::Scan3D>
  downsample(const msensor::Scan3D &input) const;

  /**
   * @brief Removes points that are closer to the scan origin than the
   * configured minimum distance.
   */
  std::shared_ptr<msensor::Scan3D>
  removePointsNearCenter(const msensor::Scan3D &input) const;

private:
  PreProcessor config_;
};
} // namespace mslam