cmake_minimum_required(VERSION 3.20)
project(mslam)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(spdlog REQUIRED)

link_libraries(Eigen3::Eigen)

set(MSLAM_CPU "native" CACHE STRING "Target CPU (e.g. cortex-a76 for Raspberry Pi 5, or native)")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  add_compile_options("-march=${MSLAM_CPU}")
else()
  add_compile_options("-mcpu=${MSLAM_CPU}")
endif()
# add_compile_definitions(EIGEN_DONT_VECTORIZE)

include(cmake/findGRPC.cmake)
include(cmake/code_format.cmake)

add_subdirectory(third_party/robin-map)
add_subdirectory(third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor EXCLUDE_FROM_ALL)
add_subdirectory(grpc)
link_libraries(msensor::msensor)

add_subdirectory(src/map)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam mslam::config msensor::server mid360 config)


add_executable(register_scans2d EXCLUDE_FROM_ALL
src/register_scans2d.cc)
target_link_libraries(register_scans2d maps slam mslam::config)

install(
	TARGETS mslam
	DESTINATION ${CMAKE_INSTALL_BINDIR}
	RUNTIME_DEPENDENCY_SET mslam_runtime_targets
)

install(
	RUNTIME_DEPENDENCY_SET mslam_runtime_targets
	PRE_INCLUDE_REGEXES "."
	POST_EXCLUDE_REGEXES "^/lib/.*" "^/usr/lib/.*"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/config/mslam.jsonc
		${CMAKE_CURRENT_SOURCE_DIR}/config/publisher_config.json
		${CMAKE_CURRENT_SOURCE_DIR}/config/mid360_config.json
	DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}
)


## Testing ##
option(MSLAM_INTEGRATION_TESTS
  "Build and register the slow end-to-end integration tests with CTest" OFF)
include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()