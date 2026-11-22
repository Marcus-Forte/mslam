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

# msensor tools (replay server, recorder): options here, targets below.
include(cmake/msensor_tools.cmake)

# SYCL is opt-in and the only parallel path. Without it mslam (and moptim) are sequential plain C++ built by
# GCC, and sycl.enabled can only be false. With it, everything that links moptim is compiled by DPC++ and
# sycl.enabled selects the SYCL kernels or the sequential host code at run time (MSLAM_SYCL=0|1 overrides
# the config).
option(MSLAM_WITH_SYCL "Build the SYCL backend (needs a DPC++ clang++)" OFF)
set(MOPTIM_WITH_SYCL ${MSLAM_WITH_SYCL})
message(STATUS "mslam: SYCL backend ${MSLAM_WITH_SYCL}")
if(MSLAM_WITH_SYCL)
  # moptim compiles fixed-size Eigen members into SYCL device code and copies them between the host and
  # the device pass, so it fixes EIGEN_MAX_STATIC_ALIGN_BYTES=16 on everything that links it. Every
  # other library here (maps, config, msensor, the I/O library) must agree on Eigen's layout, so the
  # same value is set globally, before the subdirectories, like MSLAM_CPU.
  add_compile_definitions(EIGEN_MAX_STATIC_ALIGN_BYTES=16)
endif()

add_subdirectory(third_party/robin-map)
# moptim source tree: the submodule by default. Point it at a standalone moptim checkout (e.g. the
# devcontainer's /moptim bind mount) to build mslam against a moptim working tree.
set(MSLAM_MOPTIM_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/moptim" CACHE PATH "moptim source tree")
add_subdirectory(${MSLAM_MOPTIM_SOURCE_DIR} ${CMAKE_BINARY_DIR}/third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor EXCLUDE_FROM_ALL)

# mslam uses only the namespaced msensor:: names. Dropping the legacy global
# aliases makes the compiler prove it. Defined after the msensor subdirectory,
# so msensor's own sources are not affected.
add_compile_definitions(MSENSOR_NO_LEGACY_NAMES)
add_subdirectory(grpc)
link_libraries(msensor::msensor)

add_subdirectory(src/map)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam slam_io mslam::config msensor::server mid360 config)


# playback_publisher and remote_recorder next to mslam (cmake/msensor_tools.cmake).
mslam_add_msensor_tools()


# Point-to-plane registration of whole clouds (not installed).
add_executable(register_scans
src/register_scans.cc)
target_link_libraries(register_scans maps slam slam_io mslam::config)

add_executable(register_scans2d EXCLUDE_FROM_ALL
src/register_scans2d.cc)
target_link_libraries(register_scans2d maps slam slam_io mslam::config)

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
