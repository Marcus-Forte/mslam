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
add_subdirectory(third_party/msensor_library)
link_libraries(ILidar IImu msensor::Timing msensor_library)

add_subdirectory(src/maps)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam config)

add_executable(register_scans
src/register_scans.cc)
target_link_libraries(register_scans maps slam config gl_publisher) # why does msensor_library not capture that library already?


## Testing ##
include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()