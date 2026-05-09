add_library(mslam_config
JsonConfig.cc
Validation.cc)
add_library(mslam::config ALIAS mslam_config)
target_link_libraries(mslam_config jsoncpp utils) #utils from moptim
target_include_directories(mslam_config PUBLIC ${CMAKE_SOURCE_DIR}/include)