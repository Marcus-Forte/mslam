generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/slam.proto ${CMAKE_SOURCE_DIR}/third_party/msensor/proto)

add_library(slam_proto ${CMAKE_CURRENT_BINARY_DIR}/slam.pb.cc)
target_include_directories(slam_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(slam_proto ${_PROTOBUF_LIBPROTOBUF} lidar_proto)

add_library(slam_grpc ${CMAKE_CURRENT_BINARY_DIR}/slam.grpc.pb.cc)
target_include_directories(slam_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(slam_grpc ${_GRPC_GRPCPP} slam_proto)