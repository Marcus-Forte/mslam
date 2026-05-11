cmake_minimum_required(VERSION 3.20)
project(mslam)

set(CMAKE_CXX_STANDARD 20)
find_package(PCL REQUIRED COMPONENTS kdtree search filters io)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
link_libraries(Eigen3::Eigen)

# PCL injects arch-specific flags (e.g. -march=native, -mavx2) via
# PCL_COMPILE_OPTIONS. These affect Eigen's alignment / vectorisation
# macros, so they must be applied globally to avoid ODR violations
# between translation units that share Eigen template instantiations.
add_compile_options(${PCL_COMPILE_OPTIONS})
add_compile_options("-march=native")

include(cmake/findGRPC.cmake)
include(cmake/code_format.cmake)

add_subdirectory(third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor EXCLUDE_FROM_ALL)
add_subdirectory(grpc)
link_libraries(msensor::msensor)

add_subdirectory(src/map)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam mslam::config)


add_executable(register_scans
src/register_scans.cc)
target_link_libraries(register_scans maps slam mslam::config)

# Remote recoder comes from msensor
install(TARGETS mslam remote_recorder DESTINATION ${CMAKE_INSTALL_BINDIR})


## Testing ##
include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()