#pragma once

#include "map/IMap2D.hh"

namespace mslam {
class SimpleMap2D : public IMap2D {
public:
  SimpleMap2D();
  void addScan(const PointCloud2D &points) override;

  const PointCloud2D &getPointCloudRepresentation() const override;

private:
  PointCloud2D map_;
};
} // namespace mslam