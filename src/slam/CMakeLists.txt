add_library(slam 
Registration2D.cc
Transform.cc
Slam.cc)
target_link_libraries(slam moptim)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)