#pragma once
#include "IScan.hh"
#include <fstream>
namespace mslam {

class FileScan : public IScan {
public:
  FileScan(const std::string &scan_file);
  PointCloud2T getScan(bool blocking = false) override;
  virtual IMUData getImuData() override;

private:
  std::ifstream scan_file_;
  uint64_t scan_time_;
};

} // namespace mslam