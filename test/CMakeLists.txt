find_package(GTest REQUIRED)
add_subdirectory(map)


add_executable(test_preprocessor
test_preprocessor.cc)
target_link_libraries(test_preprocessor slam GTest::gtest_main)

gtest_discover_tests(test_preprocessor)

add_executable(test_slam
test_slam.cc)
target_link_libraries(test_slam slam GTest::gtest_main)

gtest_discover_tests(test_slam)

add_executable(mslam_draft draft.cc)
target_link_libraries(mslam_draft moptim maps slam ILidar)