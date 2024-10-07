#pragma once

#include "Points.hh"

class IScan {
public:
  virtual PointCloud2D getScan() = 0;

private:
};