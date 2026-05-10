find_package(GTest REQUIRED)
add_subdirectory(map)

add_executable(test_point_metrics
test_point_metrics.cc)
target_include_directories(test_point_metrics PRIVATE ${CMAKE_SOURCE_DIR}/src/slam)
target_link_libraries(test_point_metrics slam GTest::gtest_main)

gtest_discover_tests(test_point_metrics)

add_executable(mslam_draft draft.cc)
target_link_libraries(mslam_draft moptim maps slam ILidar)