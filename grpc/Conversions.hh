#pragma once
#include "common/IMU.hh"
#include "common/Points.hh"
#include "glserver.pb.h"
#include "points.pb.h"

mslam::PointCloud2D fromGRPC(const lidar::PointCloud3 &msg);
mslam::IMUData fromGRPC(const lidar::IMUData &msg);
gl::PointCloud3 toGRPC(const mslam::PointCloud2D &pointcloud, float r = 1.0,
                       float g = 0.0, float b = 0.0);