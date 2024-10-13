# Proto in third_party/msensor_library/grpc/proto/gl_server.proto
add_library(gRPCInterface
  Conversions.cc
  RemoteScan.cc
  RemoteMap.cc
)

target_include_directories(gRPCInterface PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gRPCInterface sensors_proto sensors_grpc gl_server_grpc_client)