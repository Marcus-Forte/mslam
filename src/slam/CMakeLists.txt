add_library(slam 
registration/PointToPointRegistration.cc
registration/PointToPlaneRegistration.cc
registration/ImuRegistration.cc
registration/IcpWorkspace.cc
CorrespondenceFinder.cc
CorrespondenceFinderLogger.cc
NormalEstimator.cc
Transform.cc
Preprocessor.cc
Deskew.cc
PointCloudIO.cc
Conversion.cc
SlamServer.cc
ImuPreintegration.cc
ImuMath.cc
SensorInput.cc
ImuScanSync.cc
../utils/Timing.cc
SlamPipeline.cc
SlamRunner.cc
Slam.cc)
target_link_libraries(slam
  PUBLIC spdlog::spdlog slam_grpc maps moptim
  PRIVATE msensor scan_recorder sensors_proto)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)