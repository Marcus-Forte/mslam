#pragma once

#include "config/IConfig.hh"
#include "msensor/interface/ILidar.hh"

#include <Eigen/Dense>

namespace mslam {

class Preprocessor {
public:
  Preprocessor(const PreProcessor &config);

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

  /**
   * @brief Deskews a lidar scan assuming constant velocity motion.
   * Uses SE3 Lie group interpolation (exp/log) to properly handle
   * rotation and translation on the manifold.
   * @param scan The raw lidar scan to deskew.
   * @param relative_motion The estimated frame-to-frame SE3 delta
   *        (e.g. from the previous registration result).
   * @return A new scan with motion-corrected points.
   */
  std::shared_ptr<msensor::Scan3D>
  deskew(const msensor::Scan3D &scan,
         const Eigen::Affine3d &relative_motion) const;

  /**
   * @brief Deskews a lidar scan using the IMU-predicted motion delta.
   * Same SE3 interpolation as deskew(), but expects the delta to come
   * from integrated IMU predictions rather than the previous registration.
   * @param scan The raw lidar scan to deskew.
   * @param imu_delta The SE3 transform accumulated from IMU predictions
   *        over the inter-scan interval.
   * @return A new scan with motion-corrected points.
   */
  std::shared_ptr<msensor::Scan3D>
  deskewImu(const msensor::Scan3D &scan,
            const Eigen::Affine3d &imu_delta) const;

private:
  PreProcessor config_;
};
} // namespace mslam