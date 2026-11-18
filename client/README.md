# MSLAM client

This client subscribes to the mslam gRPC server and renders map snapshots,
transformed scans, correspondences, and poses in viser.

The LED at the top of the panel shows the connection to the SLAM server (`connection.py`): green when connected,
amber while connecting, red when unreachable (retried every 1 to 2 s, so it turns green within about 2 s of the
server coming back). It watches the gRPC channel state (an HTTP/2 connection to the address is up). It does not
check which RPCs the server implements, and it does not notice a server that stops answering without closing the
connection, such as a pulled cable.

The second LED shows the SLAM server's own link to its sensor server (`sensor_status.py`), which the SLAM server reports
through the `GetStatus` RPC (polled once a second):

- **green, streaming**: connected, and both LiDAR and IMU samples are arriving (the last of each is at most 2 s old);
- **amber, connected, no data**: connected, but samples are not arriving on both streams, for example a
  `playback_publisher` that is paused (it starts paused) or a sensor that is idle;
- **red, disconnected**: the sensor server is unreachable (the SLAM server keeps retrying every second);
- **grey, on-board sensor**: `remote_scanner` is `"local"`, so there is no link to report;
- **grey, unknown**: the viewer cannot ask (the SLAM server is unreachable, see the first LED) or the SLAM server is too
  old to implement `GetStatus`; everything else in the viewer still works against an old server.

The **Follow** dropdown keeps the sensor still on screen and moves the world around it, as rviz does with a target
frame (`follow.py`):

- **Off**: nothing moves (the view stays where following left it).
- **Position** (rviz Orbit view): only the position is followed; the view keeps its orientation.
- **Position + yaw** (rviz ThirdPersonFollower): position and heading; roll and pitch of a hand-held sensor do not
  swing the view.
- **Full pose**: position and full attitude (first person).

Turning following on points the camera at the sensor, so orbiting pivots around it; after that the camera is never
moved by the viewer, so orbit, pan and zoom work while following. Every visual sits under one `/world` frame, and
following moves that frame. The pose axes and `/world` are redrawn together at 30 Hz from the latest pose, because
poses arrive in bursts (one per IMU sample, about 10 per scan). The old Follow Pose moved the camera itself. That
shook, because viser animates a camera position set by the server over 50 ms but applies its look-at at once. It
also dragged behind the sensor, because the camera state the browser reports back lags behind what was set.

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
  the open node are assigned inside `ViserServer.atomic()` (viser, as of 1.1.1, has no single-message update).
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
