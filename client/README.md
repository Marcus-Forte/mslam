# MSLAM client

This client subscribes to the mslam gRPC server and renders map snapshots in
viser.

Generate Python stubs when the SLAM or lidar proto files change:

```bash
uv run python -m grpc_tools.protoc -I/workspaces/mslam/grpc -I/workspaces/mslam/third_party/msensor/proto --python_out=proto_gen --grpc_python_out=proto_gen --pyi_out=proto_gen /workspaces/mslam/third_party/msensor/proto/lidar.proto /workspaces/mslam/grpc/slam.proto
```

Run the viewer:

```bash
uv run slam_client.py --server-addr 127.0.0.1:50052
```