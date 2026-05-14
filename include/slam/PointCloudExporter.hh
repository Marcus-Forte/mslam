#pragma once

#include "common/Points.hh"

#include <filesystem>
#include <memory>
#include <string>

namespace mslam {

class VoxelHashMap;

class PointCloudExporter {
public:
  PointCloudExporter(std::string output_prefix, float voxel_size,
                     size_t max_points_per_voxel);

  bool isEnabled() const;
  void addTransformedScan(const PointCloud &scan);
  void save() const;

  const std::filesystem::path &getVoxelHashPath() const;
  const std::filesystem::path &getTransformedScansPath() const;
  size_t getVoxelHashPointCount() const;
  size_t getTransformedScanPointCount() const;

private:
  std::string output_prefix_;
  std::filesystem::path voxel_hash_path_;
  std::filesystem::path transformed_scans_path_;
  std::unique_ptr<VoxelHashMap> voxel_hash_map_;
  PointCloud accumulated_transformed_scans_;
};

} // namespace mslam