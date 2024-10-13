cmake_minimum_required(VERSION 3.20)
project(mslam)

set(CMAKE_CXX_STANDARD 20)
find_package(PCL REQUIRED COMPONENTS kdtree search filters)
find_package(Eigen3 REQUIRED)
link_libraries(Eigen3::Eigen)

include(cmake/findGRPC.cmake)
include(cmake/generate_proto.cmake)
include(cmake/code_format.cmake)

add_subdirectory(third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor_library EXCLUDE_FROM_ALL)
add_subdirectory(grpc)

add_subdirectory(src/map)

add_library(mslamlib
    src/FileScan.cc
    src/Registration.cc
    src/Transform.cc
)
target_include_directories(mslamlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mslamlib moptim maps sensors_proto)

add_executable(mslam
src/mslam.cc)
target_link_libraries(mslam  mslamlib gRPCInterface)


## Testing ##
include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()