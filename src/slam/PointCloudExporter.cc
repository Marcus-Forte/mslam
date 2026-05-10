#include "slam/PointCloudExporter.hh"

#include "map/VoxelHashMap.hh"

#include <filesystem>
#include <pcl/io/ply_io.h>
#include <stdexcept>

namespace {

mslam::PointCloud3 preparePointCloudForExport(const mslam::PointCloud3 &cloud) {
  auto export_cloud = cloud;
  export_cloud.width = static_cast<std::uint32_t>(export_cloud.points.size());
  export_cloud.height = 1;
  export_cloud.is_dense = false;
  return export_cloud;
}

std::filesystem::path buildPlyExportPath(const std::string &prefix,
                                         const std::string &suffix) {
  const std::filesystem::path prefix_path(prefix);
  const auto filename = prefix_path.filename();
  const auto base_name = filename.extension() == ".ply"
                             ? filename.stem().string()
                             : filename.string();
  return prefix_path.parent_path() / (base_name + "_" + suffix + ".ply");
}

void savePointCloudAsPly(const std::filesystem::path &path,
                         const mslam::PointCloud3 &cloud) {
  const auto parent_path = path.parent_path();
  if (!parent_path.empty()) {
    std::filesystem::create_directories(parent_path);
  }

  auto export_cloud = preparePointCloudForExport(cloud);
  if (pcl::io::savePLYFileBinary(path.string(), export_cloud) != 0) {
    throw std::runtime_error("Failed to save PLY file: " + path.string());
  }
}

} // namespace

namespace mslam {

PointCloudExporter::PointCloudExporter(std::string output_prefix,
                                       float voxel_size,
                                       size_t max_points_per_voxel)
    : output_prefix_(std::move(output_prefix)),
      voxel_hash_path_(buildPlyExportPath(output_prefix_, "voxel_hash")),
      transformed_scans_path_(
          buildPlyExportPath(output_prefix_, "transformed_scans")) {
  if (output_prefix_.empty()) {
    return;
  }

  voxel_hash_map_ =
      std::make_unique<VoxelHashMap>(voxel_size, max_points_per_voxel);
}

bool PointCloudExporter::isEnabled() const { return voxel_hash_map_ != nullptr; }

void PointCloudExporter::addTransformedScan(const PointCloud3 &scan) {
  if (!isEnabled()) {
    return;
  }

  voxel_hash_map_->addScan(scan);
  accumulated_transformed_scans_.points.insert(
      accumulated_transformed_scans_.points.end(), scan.points.begin(),
      scan.points.end());
}

void PointCloudExporter::save() const {
  if (!isEnabled()) {
    return;
  }

  savePointCloudAsPly(voxel_hash_path_,
                      voxel_hash_map_->getPointCloudRepresentation());
  savePointCloudAsPly(transformed_scans_path_, accumulated_transformed_scans_);
}

const std::filesystem::path &PointCloudExporter::getVoxelHashPath() const {
  return voxel_hash_path_;
}

const std::filesystem::path &
PointCloudExporter::getTransformedScansPath() const {
  return transformed_scans_path_;
}

size_t PointCloudExporter::getVoxelHashPointCount() const {
  if (!isEnabled()) {
    return 0;
  }

  return voxel_hash_map_->getPointCloudRepresentation().size();
}

size_t PointCloudExporter::getTransformedScanPointCount() const {
  return accumulated_transformed_scans_.size();
}

} // namespace mslam