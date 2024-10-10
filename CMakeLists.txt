cmake_minimum_required(VERSION 3.20)
project(mslam)

set(CMAKE_CXX_STANDARD 20)
include(cmake/format.cmake)
find_package(PCL REQUIRED COMPONENTS kdtree search filters)
find_package(Eigen3 REQUIRED)
link_libraries(Eigen3::Eigen)
option(GPRC_SENSOR "Use gRPC sensor" ON)

if(GPRC_SENSOR)
add_subdirectory(grpc)
endif()

set(BUILD_TESTING OFF)
add_subdirectory(third_party/moptim)

add_library(mslamlib
src/map/KDTree2DMap.cc
src/map/VoxelHashMap.cc
src/FileScan.cc
src/Registration.cc
src/Transform.cc)
target_include_directories(mslamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(mslamlib moptim protolib)

add_executable(mslam
src/mslam.cc)
target_link_libraries(mslam  mslamlib gRPCInterface ${PCL_LIBRARIES})