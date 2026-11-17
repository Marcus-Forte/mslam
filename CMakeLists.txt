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

add_subdirectory(third_party/robin-map)
add_subdirectory(third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor EXCLUDE_FROM_ALL)

# mslam uses only the namespaced msensor:: names. Dropping the legacy global
# aliases makes the compiler prove it. Defined after the msensor subdirectory,
# so msensor's own sources are not affected.
add_compile_definitions(MSENSOR_NO_LEGACY_NAMES)
add_subdirectory(grpc)
link_libraries(msensor::msensor)

# oneTBB runs the correspondence search and normal estimation in parallel when
# slam.num_threads (registration.num_threads for register_scans) is not 1.
# Without it the build is sequential and such a setting only logs a warning.
option(MSLAM_WITH_TBB "Use oneTBB for the parallel registration search" ON)
if(MSLAM_WITH_TBB)
  find_package(TBB QUIET)
  if(NOT TBB_FOUND)
    message(WARNING "MSLAM_WITH_TBB is ON but oneTBB was not found "
                    "(install libtbb-dev); building the sequential version")
  else()
    message(STATUS "MSLAM_WITH_TBB is ON and oneTBB was found; building the parallel version")
  endif()
endif()

add_subdirectory(src/map)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam mslam::config msensor::server mid360 config)


# playback_publisher and remote_recorder next to mslam (cmake/msensor_tools.cmake).
mslam_add_msensor_tools()


# Point-to-plane registration of whole clouds (not installed).
add_executable(register_scans
src/register_scans.cc)
target_link_libraries(register_scans maps slam mslam::config)

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
