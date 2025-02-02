add_library(config
JsonConfig.cc
Validation.cc)
target_link_libraries(config jsoncpp)
target_include_directories(config PUBLIC ${CMAKE_SOURCE_DIR}/include)