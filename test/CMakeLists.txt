find_package(GTest REQUIRED)
add_subdirectory(map)

add_executable(mslam_draft draft.cc)
target_link_libraries(mslam_draft moptim maps slam ILidar)