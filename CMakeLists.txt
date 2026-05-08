cmake_minimum_required(VERSION 3.20)
project(mslam)

set(CMAKE_CXX_STANDARD 20)
find_package(PCL REQUIRED COMPONENTS kdtree search filters)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
link_libraries(Eigen3::Eigen)

include(cmake/findGRPC.cmake)
include(cmake/generate_proto.cmake)
include(cmake/code_format.cmake)

add_subdirectory(third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor EXCLUDE_FROM_ALL)
link_libraries(ILidar IImu msensor::timing msensor)

add_subdirectory(src/maps)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam mslam::config sensors_server)

add_executable(register_scans
src/register_scans.cc)
target_link_libraries(register_scans maps slam mslam::config)


## Testing ##
include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()