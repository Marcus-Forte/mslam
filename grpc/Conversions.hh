#pragma once

#include "common/IMU.hh"
#include "common/Points.hh"
#include "gl_server.pb.h"
#include "sensors.pb.h"

mslam::PointCloud2T fromGRPC(const sensors::PointCloud3 &msg);
mslam::IMUData fromGRPC(const sensors::IMUData &msg);

gl::PointCloud3 toGRPC(const mslam::PointCloud2 &pointcloud, float r = 1.0,
                       float g = 0.0, float b = 0.0);