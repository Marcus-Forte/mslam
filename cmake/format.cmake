find_program(FORMATTER clang-format)

if(FORMATTER)
  message(STATUS "Code formatter found.")
  file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cc
    ${CMAKE_SOURCE_DIR}/include/*.hh
    ${CMAKE_SOURCE_DIR}/grpc/*.cc
    ${CMAKE_SOURCE_DIR}/grpc/*.hh)
  add_custom_target(code-format ALL
  clang-format -i ${SRC_FILES})
endif()