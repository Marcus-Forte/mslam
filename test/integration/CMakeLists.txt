# End-to-end integration tests: replay a recorded .pbscan through the full SLAM
# pipeline. They are slow, so they are opt-in:
#
#   cmake --preset gcc -DMSLAM_INTEGRATION_TESTS=ON
#   cmake --build --preset build-gcc
#   ctest --test-dir build/gcc -L integration
#
# With the option OFF (the default) they are neither built by the default build
# nor registered with CTest, so a plain `ctest` never runs them. They can still
# be built and run directly, with or without the option:
#
#   cmake --build --preset build-gcc --target integration-test
if(MSLAM_INTEGRATION_TESTS)
  add_executable(test_slam_integration
    test_slam_integration.cc)
else()
  add_executable(test_slam_integration EXCLUDE_FROM_ALL
    test_slam_integration.cc)
endif()

target_link_libraries(test_slam_integration
  slam maps mslam::config msensor::server scan_recorder GTest::gtest_main)

# The recordings and config live in the repository, not the build tree.
target_compile_definitions(test_slam_integration
  PRIVATE MSLAM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if(MSLAM_INTEGRATION_TESTS)
  gtest_discover_tests(test_slam_integration
    PROPERTIES
      LABELS integration)
endif()

# Build and run directly, without going through CTest.
add_custom_target(integration-test
  COMMAND $<TARGET_FILE:test_slam_integration>
  DEPENDS test_slam_integration
  USES_TERMINAL
  COMMENT "Running mslam integration tests")
