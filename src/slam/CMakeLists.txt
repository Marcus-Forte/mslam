add_library(slam 
Registration2D.cc
Transform.cc
SlamPlayer.cc
Slam.cc)
target_link_libraries(slam moptim msensor_library)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)