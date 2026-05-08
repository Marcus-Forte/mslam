find_program(FORMATTER clang-format)

if(FORMATTER)
  file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/**/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/**/*.hh
  )

  list(FILTER SRC_FILES EXCLUDE REGEX "/third_party/")
  list(FILTER SRC_FILES EXCLUDE REGEX "/build/")

  add_custom_target(format-mslam ALL
    clang-format -i ${SRC_FILES})
endif()