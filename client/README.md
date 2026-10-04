# MSLAM client

This client subscribes to the mslam gRPC server and renders map snapshots,
transformed scans, correspondences, and poses in viser.

Enable **Follow Pose** in the viewer to move and rotate the camera with the SLAM
pose while preserving the current camera offset. You can still orbit the camera;
its position and rotation remain relative to the followed pose.

Generate Python stubs when the SLAM or lidar proto files change:

```bash
uv run python -m grpc_tools.protoc -I../grpc -I../third_party/msensor/proto --python_out=proto_gen --grpc_python_out=proto_gen --pyi_out=proto_gen ../third_party/msensor/proto/lidar.proto ../grpc/slam.proto
```

Run the viewer:

```bash
uv run slam_client.py --server-addr 127.0.0.1:50052
```