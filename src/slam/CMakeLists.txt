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
target_link_libraries(slam maps moptim msensor scan_recorder sensors_proto slam_grpc mslam::mlog)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)