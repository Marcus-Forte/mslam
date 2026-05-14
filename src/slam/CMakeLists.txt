add_library(slam 
Registration.cc
CorrespondenceFinder.cc
Transform.cc
Preprocessor.cc
PointCloudExporter.cc
Conversion.cc
SlamServer.cc
RecordingSensorPlayer.cc
Slam.cc)
target_link_libraries(slam maps moptim msensor scan_recorder sensors_proto slam_grpc)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)