add_library(maps
  KDTreeMap.cc
  OctreeMap.cc
  VoxelHashMap.cc
  Voxel.cc
)
target_include_directories(maps PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(maps pcl_filters)