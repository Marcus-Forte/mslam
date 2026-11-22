add_library(slam 
registration/PointToPointRegistration.cc
registration/PointToPlaneRegistration.cc
registration/ImuRegistration.cc
registration/IcpWorkspace.cc
registration/DeviceCorrespondenceSearch.cc
DeviceVoxelHashMap.cc
DevicePreprocessor.cc
ComputeContext.cc
CorrespondenceFinder.cc
CorrespondenceFinderLogger.cc
NormalEstimator.cc
Transform.cc
Preprocessor.cc
Deskew.cc
ScanSequenceRegistration.cc
ImuPreintegration.cc
ImuMath.cc
SensorInput.cc
ImuScanSync.cc
../utils/Timing.cc
SlamPipeline.cc
SlamRunner.cc
Slam.cc
MapFactory.cc
posegraph/PoseGraphOptimizer.cc
posegraph/KeyframeBuilder.cc
posegraph/LoopCloser.cc
posegraph/PoseGraphBackend.cc)
target_link_libraries(slam
  PUBLIC spdlog::spdlog maps moptim
  PRIVATE msensor)
target_include_directories(slam PUBLIC ${CMAKE_SOURCE_DIR}/include)

# gRPC server, protobuf conversions and point-cloud files. Kept out of `slam`: they don't link moptim,
# so they are not compiled with -fsycl (no SYCL device pass over the protobuf/abseil/gRPC headers).
add_library(slam_io
SlamServer.cc
Conversion.cc
PointCloudIO.cc)
target_link_libraries(slam_io
  PUBLIC spdlog::spdlog slam_grpc
  PRIVATE msensor scan_recorder sensors_proto)
target_include_directories(slam_io PUBLIC ${CMAKE_SOURCE_DIR}/include)
