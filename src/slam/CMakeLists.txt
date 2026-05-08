add_library(slam 
Registration.cc
Transform.cc
Preprocessor.cc
SlamPlayer.cc
Slam.cc)
target_link_libraries(slam moptim msensor scan_recorder sensors_proto)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)