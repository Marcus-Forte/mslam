#pragma once
#include "common/Points.hh"
#include "glserver.pb.h"
#include "points.pb.h"

mslam::PointCloud2D fromGRPC(const lidar::PointCloud3 &msg);
gl::PointCloud3 toGRPC(const mslam::PointCloud2D &pointcloud);