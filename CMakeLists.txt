cmake_minimum_required(VERSION 3.20)
project(mslam)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

link_libraries(Eigen3::Eigen)

add_compile_options("-march=native")
# add_compile_definitions(EIGEN_DONT_VECTORIZE)

include(cmake/findGRPC.cmake)
include(cmake/code_format.cmake)

add_subdirectory(third_party/robin-map)
add_subdirectory(third_party/moptim EXCLUDE_FROM_ALL)
add_subdirectory(third_party/msensor EXCLUDE_FROM_ALL)
add_subdirectory(grpc)
link_libraries(msensor::msensor)

# Keep the full msensor subproject out of the default build, but include the
# recorder binary so the top-level install target can stage it.
set_property(TARGET remote_recorder PROPERTY EXCLUDE_FROM_ALL FALSE)

add_subdirectory(src/map)
add_subdirectory(src/config)
add_subdirectory(src/slam)

add_executable(mslam
src/main.cc)
target_link_libraries(mslam maps slam mslam::config msensor::server)


add_executable(register_scans2d EXCLUDE_FROM_ALL
src/register_scans2d.cc)
target_link_libraries(register_scans2d maps slam mslam::config)

install(
	TARGETS mslam remote_recorder
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
	FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/mslam.jsonc
	DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}
)


## Testing ##
include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()