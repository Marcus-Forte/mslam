find_package(GTest REQUIRED)
add_subdirectory(map)
add_subdirectory(integration)


add_executable(test_preprocessor
test_preprocessor.cc)
target_link_libraries(test_preprocessor slam GTest::gtest_main)

gtest_discover_tests(test_preprocessor)

add_executable(test_deskew
test_deskew.cc)
target_link_libraries(test_deskew slam GTest::gtest_main)

gtest_discover_tests(test_deskew)

add_executable(test_point_cloud_io
test_point_cloud_io.cc)
target_link_libraries(test_point_cloud_io slam slam_io scan_recorder GTest::gtest_main)

gtest_discover_tests(test_point_cloud_io)

add_executable(test_scan_sequence_registration
test_scan_sequence_registration.cc)
target_link_libraries(test_scan_sequence_registration slam maps GTest::gtest_main)

gtest_discover_tests(test_scan_sequence_registration)

add_executable(test_registration_config
test_registration_config.cc)
target_link_libraries(test_registration_config mslam::config GTest::gtest_main)
# The shipped example config is parsed by one test.
target_compile_definitions(test_registration_config
  PRIVATE MSLAM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

gtest_discover_tests(test_registration_config)

add_executable(test_correspondence_finder
test_correspondence_finder.cc)
target_link_libraries(test_correspondence_finder slam GTest::gtest_main)

gtest_discover_tests(test_correspondence_finder)

add_executable(test_imu_math
test_imu_math.cc)
target_link_libraries(test_imu_math slam GTest::gtest_main)

gtest_discover_tests(test_imu_math)

add_executable(test_slam
test_slam.cc)
target_link_libraries(test_slam slam GTest::gtest_main)

gtest_discover_tests(test_slam)

add_executable(test_json_config
test_json_config.cc)
target_link_libraries(test_json_config mslam::config GTest::gtest_main)
target_compile_definitions(test_json_config
  PRIVATE MSLAM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

gtest_discover_tests(test_json_config)

add_executable(test_point_distance
test_point_distance.cc)
target_link_libraries(test_point_distance slam moptim GTest::gtest_main)

gtest_discover_tests(test_point_distance)

add_executable(test_se3_manifold
test_se3_manifold.cc)
target_link_libraries(test_se3_manifold slam moptim GTest::gtest_main)

gtest_discover_tests(test_se3_manifold)

add_executable(test_imu_preintegration_factor
test_imu_preintegration_factor.cc)
target_link_libraries(test_imu_preintegration_factor slam moptim GTest::gtest_main)

gtest_discover_tests(test_imu_preintegration_factor)

add_executable(test_normal_estimator
test_normal_estimator.cc)
target_link_libraries(test_normal_estimator slam maps GTest::gtest_main)

gtest_discover_tests(test_normal_estimator)

add_executable(test_registration
test_registration.cc)
target_link_libraries(test_registration slam maps GTest::gtest_main)

gtest_discover_tests(test_registration)

add_executable(test_transform
test_transform.cc)
target_link_libraries(test_transform slam GTest::gtest_main)

gtest_discover_tests(test_transform)

add_executable(test_imu_scan_sync
test_imu_scan_sync.cc)
target_link_libraries(test_imu_scan_sync slam GTest::gtest_main)

gtest_discover_tests(test_imu_scan_sync)

add_executable(test_slam_pipeline
test_slam_pipeline.cc)
target_link_libraries(test_slam_pipeline slam maps GTest::gtest_main)

gtest_discover_tests(test_slam_pipeline)

add_executable(test_slam_server
test_slam_server.cc)
target_link_libraries(test_slam_server slam slam_io GTest::gtest_main)

gtest_discover_tests(test_slam_server)

add_executable(test_timing
test_timing.cc)
target_link_libraries(test_timing slam GTest::gtest_main)

gtest_discover_tests(test_timing)

add_executable(test_device_preprocessor
test_device_preprocessor.cc)
target_link_libraries(test_device_preprocessor slam GTest::gtest_main)

gtest_discover_tests(test_device_preprocessor)
if(MSLAM_WITH_SYCL)
  gtest_discover_tests(test_device_preprocessor TEST_SUFFIX .host
    PROPERTIES ENVIRONMENT MOPTIM_BACKEND=host)
endif()

add_executable(test_device_correspondences
test_device_correspondences.cc)
target_link_libraries(test_device_correspondences slam maps GTest::gtest_main)

gtest_discover_tests(test_device_correspondences)
if(MSLAM_WITH_SYCL)
  # The device components on moptim's host backend too (their default context).
  gtest_discover_tests(test_device_correspondences TEST_SUFFIX .host
    PROPERTIES ENVIRONMENT MOPTIM_BACKEND=host)
endif()

add_executable(test_parallel
test_parallel.cc)
target_link_libraries(test_parallel slam GTest::gtest_main)

gtest_discover_tests(test_parallel)

add_executable(mslam_draft draft.cc)
target_link_libraries(mslam_draft moptim maps slam ILidar)

# Offline replay benchmark (not registered with CTest, built by default):
#   cmake --build --preset build-gcc --target bench_slam
add_executable(bench_slam bench_slam.cc)
target_link_libraries(bench_slam slam slam_io maps mslam::config scan_recorder)

add_executable(test_pose_graph_optimizer
test_pose_graph_optimizer.cc)
target_link_libraries(test_pose_graph_optimizer slam GTest::gtest_main)

gtest_discover_tests(test_pose_graph_optimizer)

add_executable(test_pose_graph_keyframes
test_pose_graph_keyframes.cc)
target_link_libraries(test_pose_graph_keyframes slam GTest::gtest_main)

gtest_discover_tests(test_pose_graph_keyframes)

add_executable(test_pose_graph_backend
test_pose_graph_backend.cc)
target_link_libraries(test_pose_graph_backend slam slam_io GTest::gtest_main)
target_compile_definitions(test_pose_graph_backend
  PRIVATE MSLAM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

gtest_discover_tests(test_pose_graph_backend)
