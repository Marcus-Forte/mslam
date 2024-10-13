add_library(maps
  KDTree2DMap.cc
  VoxelHashMap.cc
  Voxel.cc
)
target_include_directories(maps PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(maps pcl_filters)