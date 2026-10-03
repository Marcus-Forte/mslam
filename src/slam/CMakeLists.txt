add_library(slam 
registration/PointToPointRegistration.cc
registration/PointToPlaneRegistration.cc
registration/ImuRegistration.cc
CorrespondenceFinder.cc
NormalEstimator.cc
Transform.cc
Preprocessor.cc
PointCloudIO.cc
Conversion.cc
SlamServer.cc
RecordingSensorPlayer.cc
ImuPreintegration.cc
Slam.cc)
target_link_libraries(slam
  PUBLIC spdlog::spdlog slam_grpc
  PRIVATE maps moptim msensor scan_recorder sensors_proto)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)