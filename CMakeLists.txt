cmake_minimum_required(VERSION 3.20)
project(mslam)

set(CMAKE_CXX_STANDARD 20)
include(cmake/format.cmake)

find_package(Eigen3 REQUIRED)
link_libraries(Eigen3::Eigen)
option(GPRC_SENSOR "Use gRPC sensor" ON)

if(GPRC_SENSOR)
add_subdirectory(grpc)
endif()

set(BUILD_TESTING OFF)
add_subdirectory(third_party/moptim)


add_executable(mslam src/app.cc)
target_link_libraries(mslam moptim RemoteScan)