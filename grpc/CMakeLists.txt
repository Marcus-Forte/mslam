# Add .cmake as modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(generate_proto)
include(findGRPC)

if(gRPC_FOUND)
    message(STATUS "gRPC detected.")
    generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/points.proto)
    generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/proto/glserver.proto)
    add_library(protolib 
    ${CMAKE_CURRENT_BINARY_DIR}/points.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/points.grpc.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/glserver.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/glserver.grpc.pb.cc)
    target_include_directories(protolib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(protolib ${_GRPC_GRPCPP})
endif()

add_library(gRPCInterface
Conversions.cc
RemoteScan.cc
RemoteMap.cc)

target_include_directories(gRPCInterface PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gRPCInterface protolib)