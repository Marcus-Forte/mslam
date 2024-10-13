find_program(FORMATTER clang-format)

if(FORMATTER)
  file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/**/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/**/*.hh
)
add_custom_target(format-mslam ALL
clang-format -i ${SRC_FILES})
endif()