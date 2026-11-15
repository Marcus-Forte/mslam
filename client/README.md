# MSLAM client

This client subscribes to the mslam gRPC server and renders map snapshots,
transformed scans, correspondences, and poses in viser.

Enable **Follow Pose** in the viewer to move and rotate the camera with the SLAM
pose while preserving the current camera offset. You can still orbit the camera;
its position and rotation remain relative to the followed pose.

Generate Python stubs when the SLAM or lidar proto files change:

```bash
uv run python -m grpc_tools.protoc -I../grpc -I../third_party/msensor/proto --python_out=proto_gen --grpc_python_out=proto_gen --pyi_out=proto_gen ../third_party/msensor/proto/header.proto ../third_party/msensor/proto/lidar.proto ../grpc/slam.proto
```

(`lidar.proto` imports `header.proto`, so all three are generated.)

Run the viewer:

```bash
uv run slam_client.py --server-addr 127.0.0.1:50052
```

## Map stream

The viewer builds its map from `StreamMap` (contract in `../grpc/slam.proto` and the "Map stream" section of
`../readme.md`). There is no fallback to `GetMap` / `GetMapIncrements`: against a server without `StreamMap` the
viewer logs one error ("too old for this viewer") and keeps retrying at the normal interval.

- `map_store.py` is the model (no viser, no gRPC). `MapStore.apply(chunk)` implements the client side of the
  contract and returns a `MapChange`. Points live in blocks of 200,000: points are appended to one open block, a full
  block is sealed and never modified again, so an append costs time proportional to the appended points.
- `map_stream.py` runs one `StreamMap` call against a store (`stream_map`).
- `map_renderer.py` shows a store in viser. Each sealed block is one point-cloud node, created once. The open block is
  one node, re-sent when it changed, at most twice per second (a flush thread; many chunks between two flushes cause one
  update), so the data sent per update is bounded by the block size however large the map is. Points and colours of
  the open node are assigned inside `ViserServer.atomic()` (viser 1.0.16 has no single-message update).
- Colours are per point and independent of how the stream was chunked: intensity over the fixed range 0 to 255
  (`points.INTENSITY_RANGE`), or, without intensity, height over a fixed range (`points.HEIGHT_RANGE_M`).

Resume and reset:

- Every request carries the stored `(epoch, next_offset)`. After a stream error the viewer reconnects (every 5 s) and
  continues from there; the local map stays on screen while disconnected. A new server process or a reset changes the
  epoch, and the map is cleared when the stream announces it.
- The **Reset** button calls the `Reset` RPC and clears the scan, correspondence and pose visuals. The map clears
  when the stream announces the new epoch, so every connected viewer clears, not only the one that clicked.
- A chunk of the same epoch that starts before the offset already applied is a protocol violation: it is logged once,
  the map is cleared and the stream restarts from offset 0 (epoch 0 in the request).
- Per-chunk and per-scan messages are logged at DEBUG (`--log-level DEBUG`); connection messages stay at INFO.

Tests (standard library `unittest`, no server or browser needed), from this directory:

```bash
uv run python -m unittest
```

End-to-end check against a running mslam, headless (map model and generated stub only). It starts consumer A at
once, B when A is about half way, and C at once with a disconnect and resume part-way. It prints "ready", then you
start the playback; at the end A, B and C must hold the same epoch, offset, point count and checksums, and after a
`Reset` all must see a new epoch and an empty map. Exit status 0 means it passed.

```bash
uv run python tools/check_map_stream.py --server 127.0.0.1:50052 [--half-offset 500] [--disconnect-offset 200]
```
