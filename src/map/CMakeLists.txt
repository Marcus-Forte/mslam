add_library(maps
  VoxelHashMap.cc
  Voxel.cc
)
target_include_directories(maps PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(maps PUBLIC tsl::robin_map)