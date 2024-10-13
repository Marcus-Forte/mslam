
add_executable(test_voxel_hash
test_voxel_hash.cc)
target_link_libraries(test_voxel_hash maps GTest::gtest_main)

gtest_discover_tests(test_voxel_hash)