# mslam: Minimal Slam by Marcus

## Configuration

Check `config/mslam.jsonc` for the current SLAM configuration. The main SLAM binary loads this JSON file at startup. The configuration includes: 
- which sensors to use (IMU, LiDAR, camera)
- sensor source (local or remote)
- logging level
- map type (voxel or kdtree)
- remote scanner address for live SLAM
- scan preprocessing parameters (downsampling, deskewing)
- map resolution

### Configure log level

Set the top-level `log_level` field in the configuration file passed with `-c`.
Supported values are `trace`, `debug`, `info`, `warning`, and `error`.
`trace` produces the most output; `error` produces the least. For example, set
the field to `"info"` to show informational messages and more severe messages:

```json
{
  "log_level": "info"
}
```

Restart `mslam` after changing the value for the new level to take effect.

### Threads

`slam.num_threads` (optional, default 1) sets how many threads run the
correspondence search and the surface normal estimation during registration:
1 runs sequentially, 0 uses every hardware thread, n uses at most n (the SLAM
thread included). The pose does not depend on it: the result is the same bit for
bit at any thread count. `register_scans` reads the same setting from
`registration.num_threads` in its own config.

The parallel code uses oneTBB (`libtbb-dev` to build, `libtbb12` at run time).
CMake looks for it when `MSLAM_WITH_TBB` is ON (the default); if it is not
found, or with `-DMSLAM_WITH_TBB=OFF`, mslam is built sequential and a
`num_threads` other than 1 logs a warning and runs sequentially.

```json
{
  "slam": { "num_threads": 4 }
}
```

### Compute backend

`slam.backend` (optional, default `"host"`) selects where the registration and
the scan preprocessing run:

- `"host"`: the CPU code: the `VoxelHashMap` correspondence search and normals
  (on oneTBB with `num_threads`), the host preprocessing, and moptim's
  Levenberg-Marquardt with its costs on moptim's host backend (the calling
  thread, or a thread pool for large costs). Every build has it.
- `"sycl"`: SYCL kernels on moptim's SYCL device (`native_cpu`, the host CPU,
  for now). A device mirror of the voxel map keeps a normal cache across scans.
  Each ICP round runs one kernel for the correspondence search and normals,
  each scan one kernel for its preprocessing, and each round's whole
  Levenberg-Marquardt solve is one launch (with `debug` logging it uses the host
  solver, whose telemetry needs an observer). It needs a build with
  `-DMSLAM_WITH_SYCL=ON` (the `sycl` preset, DPC++); a build without SYCL
  refuses `"sycl"` at startup.

The environment variable `MSLAM_BACKEND=host|sycl` overrides the config file,
and `bench_slam` and `register_scans` take `--backend host|sycl`. Both backends
give the same pose at any thread count. The two backends' poses agree to a
fraction of a millimetre: the device kernels evaluate the normals with scalar
Eigen. `test/bench_backends.py` compares builds and backends (see
[Benchmark](#benchmark)).

```json
{
  "slam": { "backend": "sycl" }
}
```

### Strategy

`slam.strategy` (optional) selects the estimator: `"odometry"` (default, the
LiDAR-inertial odometry alone) or `"pose_graph"` (the same odometry plus a
keyframe pose-graph back end with loop closure). An unknown value is rejected.
The optional top-level `pose_graph` block is read only for `pose_graph`; every
key is optional:

| Key | Default | Meaning |
| --- | --- | --- |
| `keyframe_distance` | 0.5 m | movement that opens a new keyframe |
| `keyframe_angle` | 0.26 rad | rotation that opens a new keyframe |
| `keyframe_max_interval_s` | 0 (off) | closes a keyframe after this long, even when still |
| `mini_map_voxel_size` | 0.05 m | thinning of a keyframe's mini map, and the voxel a keyframe owns |
| `loop_search_radius` | 2.0 m | loop candidates within this distance |
| `loop_min_separation` | 10 | keyframes skipped as recent neighbours |
| `loop_submap_keyframes` | 2 | neighbours on each side of a candidate in the verification target |
| `loop_max_rms` | 0.05 m | loop accepted only at or below this RMS |
| `loop_min_matched_fraction` | 0.6 | loop accepted only at or above this matched fraction, in (0, 1] |
| `loop_sigma_translation` / `loop_sigma_rotation` | 0.02 m / 0.005 rad | loop edge noise |
| `odometry_sigma_translation` / `odometry_sigma_rotation` | 0.05 m / 0.01 rad | odometry edge noise |
| `optimizer_iterations` | 20 | pose-graph optimizer iterations |
| `synchronous` | false | run the back end inline (deterministic replays) |

## Docker

Build and run from the repository root:

```bash
docker build -f deployment/Dockerfile -t mslam .
docker run -it --rm -v ./config/:/config/ mslam mslam -c /config/mslam.json
```

If you want a custom configuration, mount your JSON file into the container and
pass that path with `-c`.

The image is built with the `pi5` preset: GCC for the Cortex-A76, host backend
only. `--build-arg PRESET=pi5-sycl` builds it with DPC++ and both backends
(`slam.backend` then selects one).

For the Python viewer client.
```bash
docker build -f deployment/DockerfileViewer -t mslam-viewer .
docker run --rm -it mslam-viewer --server-addr <address>
```

## Running playback

Replay a recording by running the replay server (from `msensor`) and pointing
the SLAM at it as a remote sensor server:

```bash
# Terminal 1: serve the recording (speed 1.0 = real time, 0 = as fast as possible)
./build/gcc/playback_publisher -f test/data/rotate.pbscan -s 1.0 -p 50051

# Terminal 2: run SLAM against the replay server
./build/gcc/mslam -c config/mslam.jsonc   # remote_scanner points at 127.0.0.1:50051
```

`playback_publisher` arguments:

- `-f, --file <file>` the recorded `.pbscan` to replay.
- `-s, --speed <x>` playback speed multiplier (`1.0` = real time, `0` = unlimited).
- `-p, --port <port>` gRPC listen port (default `50051`).
- `-a, --autoplay` start immediately instead of waiting for Space (useful for
  Docker, CI, and background processes without interactive stdin).

Playback starts paused unless `--autoplay` is supplied. Use Space to toggle
play/pause, `r` to rewind and pause, Right Arrow to double the speed, and Left
Arrow to halve it (down to 0.125x). Speed `0` means unpaced; Left Arrow changes
that to 64x. Ctrl-C stops the server. The server stays open at end-of-file, and
SLAM remains running with no incoming scans; press `r` and then Space to replay.
Keyboard controls require stdin to remain open; use `--autoplay` for
non-interactive playback.

Set `remote_scanner` to `"local"` in the SLAM configuration to use a local
Mid360. Its enable flag, SDK config path, and scan accumulation count are read
from `config/publisher_config.json` (or the `publisher_config.json` beside the
SLAM config passed with `-c`). Relative Mid360 config paths are resolved
relative to that file. The repository's sensor configs are installed alongside
the SLAM config.

If the gRPC connection drops, the remote client retries while the SLAM loop
continues running. This lets playback resume after a reset or server restart.

## Recording

`remote_recorder` (from `msensor`) records the LiDAR and IMU streams of a
sensor server into a `.pbscan` that `playback_publisher` and `bench_slam` can
replay:

```bash
cmake --build --preset build-gcc --target remote-recorder
./build/gcc/remote_recorder -a 192.168.178.129:50051 -o my_run.pbscan   # Ctrl-C stops and closes the file
```

Like the replay server, it is installed only with
`-DMSLAM_INSTALL_REMOTE_RECORDER=ON` (on in the `gcc` preset, off by default
and in the `pi5` preset).

## Browser viewer

The `viewer/` folder contains a minimal TypeScript browser app for inspecting
local `.ply` files (a static viewer sized for about 10 million points, not a
live stream listener).

```bash
cd viewer
npm ci
npm run dev -- --host
npm test
```

Run the install, build and tests in the devcontainer: `node_modules/` is
installed for its Linux platform, so do not run `npm ci` on the host over it.

The viewer supports:

- drag-and-drop or file-picker loading for local `.ply` files
- Z-up camera orientation
- point-size adjustment
- multiple ruler measurements with clear-all
- `W`, `A`, `S`, `D` camera movement

PLY files are parsed in a Web Worker. Binary little-endian files with a single
`vertex` element of scalar properties (what `bench_slam --save-map` and
`register_scans -o` write) take a fast typed-array path; any other PLY (ASCII,
big-endian, faces) falls back to three's `PLYLoader`. The `intensity` colours
are baked once into 8-bit RGB, and the scene is rendered on demand rather than
every display refresh. See `viewer/README.md`.

## Registration metrics

3D registration has a point-to-point and a point-to-plane implementation
(`PointToPointRegistration`, `PointToPlaneRegistration`). The main SLAM path uses
point-to-plane: both `PointToPlaneRegistration` and the IMU-aided
`ImuRegistration` use `Point3PlaneDistance`, a scalar point-to-plane residual
with the analytical jacobian with respect to a left se(3) increment, through
`moptim::AnalyticalCost`. The optimizer variable is the pose itself, stepped by
`SE3Manifold` (a moptim manifold). Point-to-point still uses numerical
differentiation.

## Timing

The runtime now logs timings for several parts of the SLAM pipeline, including:

- correspondence search and optimization inside 3D registration
- scan preprocessing
- SLAM update / registration
- scan transform
- map and server publication

## Registering individual scans

`register_scans` registers whole point clouds, in the order given on the command
line, with the point-to-plane registration the SLAM uses. It is built with the
default target and is not installed:

```bash
cmake --build --preset build-gcc --target register_scans
./build/gcc/register_scans [-c registration.jsonc] [--poses poses.jsonc] \
    [-o merged.ply] [--save-poses out.jsonc] a.ply b.ply c.pbscan
```

- Inputs are `.ply` files or `.pbscan` recordings. A recording is read as one
  cloud: the points of every LiDAR scan are concatenated as recorded (IMU
  entries are skipped, nothing is deskewed), so it only makes sense for a
  static recording.
- The first cloud defines the common frame: it is placed at identity (or at its
  pose from `--poses`) and is not registered. Every later cloud is registered
  against a map holding all clouds registered so far, and its result is the
  pose of its own frame in the common frame, printed as
  `pose=[x y z roll pitch yaw]` (m, rad; `R = Rx(roll) * Ry(pitch) * Rz(yaw)`).
- Each registration starts from the pose estimated for the previous cloud
  (identity for the second cloud), or from the cloud's measured pose when
  `--poses` lists it. A step converges when its starting pose is within about
  `max_correspondence_distance` of the truth, so give measured poses for clouds
  that are further apart than that.
- One line per cloud goes to stdout: points read and used (after thinning),
  `reference` or `prior=measured|estimated`, matched points and percentage,
  point-to-plane RMS in m against the map before the cloud joined it, the
  first-round correction in mm / mrad, and the pose. A step whose first round
  found no correspondences is flagged on its line, keeps its prior pose and
  makes the exit code 2. Logging goes to stderr.
- `-o merged.ply` writes all clouds at full resolution in the common frame (a
  binary PLY the `viewer/` reads). `--save-poses out.jsonc` writes the poses in
  the `--poses` format, so a result can be passed back for a finer run. Both
  files are opened before any cloud is loaded.

`-c` selects the tuning file (`config/registration.jsonc`; every key is
optional, unknown keys and invalid values are rejected):

```jsonc
{
    "log_level": "info",
    "voxel_size": 0.05,          // every cloud is thinned to voxel centroids (m)
    "map": { "resolution": 0.1, "max_points_per_voxel": 8 },
    "registration": {
        "reg_iterations": 30,
        "optimizer_iterations": 5,
        "max_correspondence_distance": 0.3,
        "num_threads": 1             // 0 = all hardware threads; same poses at any value
    }
}
```

The map's neighbour search radius is derived from the gate:
`ceil(max_correspondence_distance / map.resolution)` voxels (at most 8, with a
warning when clamped).

`--poses` is optional. It is a flat JSON object (comments allowed) from input
file name to `[x, y, z, roll, pitch, yaw]`, the pose of that cloud's frame in
the common frame:

```jsonc
{
    "b.ply":    [2.10, 0.00, 0.0, 0, 0, 1.5708],
    "c.pbscan": [2.10, 3.40, 0.0, 0, 0, 3.1416]
    // clouds not listed start from the previous estimate
}
```

A key matches an input if it equals the argument as typed or its file name. A
key that matches no input or more than one, and a value that is not six finite
numbers, is an error.

`register_scans2d` (`--target register_scans2d`, not built by default) is a
small 2D point-to-point utility for pairwise experiments on `.ply` scans:
`./build/gcc/register_scans2d scan1.ply scan2.ply`.

## Inspecting generated SIMD instructions

The project compiles with `-march=native` (x86) / `-mcpu=native` (ARM) by default, set via the `MSLAM_CPU` cache variable (the `pi5` preset and the Docker image use `cortex-a76`). This allows the compiler to emit
AVX/FMA instructions for Eigen operations. To verify which source lines produce
SIMD code, disassemble an object file with line annotations:

```bash
# With source-line annotations (requires debug info, i.e. -g):
objdump -d -l build/default/src/slam/CMakeFiles/slam.dir/Transform.cc.o

# With interleaved source code:
objdump -d -S build/default/src/slam/CMakeFiles/slam.dir/Transform.cc.o

# Filter for SIMD instructions only:
objdump -d build/default/src/slam/CMakeFiles/slam.dir/Transform.cc.o \
  | grep -E "vmov|vadd|vmul|vfma|vbroadcast"

# On ARM64 (NEON/SVE):
objdump -d build/default/src/slam/CMakeFiles/slam.dir/Transform.cc.o \
  | grep -E "fmla|fmul|fadd|ld1|st1|fmadd|fmsub"
```

The key functions to inspect:

| Object file | Source | What it does |
|---|---|---|
| `Transform.cc.o` | `transformCloud(Affine3d, ...)` | Per-point affine transform (hot loop, uses AVX2 ymm registers) |
| `Transform.cc.o` | `toAffine(..., rx, ry, rz)` | Rotation matrix construction from Euler angles |
| `PointToPlaneRegistration.cc.o` | `Align3D` | 6×6 normal equation solve |
| `NormalEstimator.cc.o` | `estimate` | 3×3 covariance eigensolver |

Replace the path after `CMakeFiles/slam.dir/` with any source file to inspect
other translation units.

## Benchmark

`bench_slam` replays a recording offline and deterministically (no sensor
threads, no pacing) through the same `SlamPipeline` the live loop uses, and
prints the time per stage and the final pose:

```bash
cmake --build --preset build-gcc --target bench_slam
build/gcc/test/bench_slam test/integration/mslam.jsonc test/data/living_room_garden.pbscan 5
```

Each line reports the number of scans, the total time and the time spent in
preprocessing, registration, map insertion and IMU prediction (ms), then the
final pose with 9 digits. Two more lines give the registration statistics
(scans through the IMU path vs LiDAR-only, mean size of the first registration
round's pose increment in mm / mrad, integrated IMU time) and the IMU/scan
synchronisation counters. The pose is identical from run to run, so a change
that must not alter the result can be checked by comparing it; the machine is
noisy, so compare the minimum over several runs, alternating old and new
binaries. It is not part of CTest.

`--save-map <file.ply>` (before or after the positional arguments) writes the
final accumulated map: every map increment the pipeline published,
concatenated, which is what `SlamServer::GetMap` serves live (not the bounded
registration map). The file is a binary little-endian PLY with
`x y z intensity` floats that the TypeScript viewer in `viewer/` opens. With
`repeats` greater than 1 the map of the last run is written. The file is
opened before the first run, so an unwritable path fails immediately; the
default run, without the option, is unchanged.

```bash
build/gcc/test/bench_slam test/integration/mslam.jsonc test/data/living_room_garden.pbscan 1 --save-map map.ply
```

`--threads <n>` overrides `slam.num_threads` of the config (1 sequential, 0
every hardware thread). The pose must be the same at every value, so it is
also a check of the parallel code:

```bash
build/gcc/test/bench_slam test/integration/mslam.jsonc test/data/living_room_garden.pbscan 5 --threads 4
```

`--backend host|sycl` overrides `slam.backend` (see
[Compute backend](#compute-backend)). The first line of the output names the
backend and whether the build has SYCL.

### Comparing builds and backends

`test/bench_backends.py` (Python 3, no dependencies) runs `bench_slam`
binaries side by side. It runs them interleaved, in a rotated order every
round. For each recording and thread count it prints each label's minimum
total, register, search and solve times, the ratio to a reference label, the
moptim launches, the final pose, and that pose's distance from the
reference's. Each `--run` is `LABEL=BINARY[:BACKEND]`, so one SYCL build can
be listed twice, once per backend:

```bash
test/bench_backends.py --run host=build/gcc/test/bench_slam \
    --run sycl=build/sycl/test/bench_slam:sycl --run sycl-host=build/sycl/test/bench_slam:host \
    --threads 1 4 --rounds 5 test/data/living_room_garden.pbscan test/data/rooms.pbscan
```

It exits with status 1 in any of these cases:

- a run fails;
- a label's pose changes between rounds or thread counts;
- a pose is further from the reference's than `--tolerance-mm` /
  `--tolerance-mrad` (defaults 0.5 mm and 0.15 mrad).

Other options:

- `--ref` names the reference label (the first `--run` by default).
- `--strategy` overrides `slam.strategy`.
- `--out <dir>` keeps every raw output.
- `--history <file.jsonl>` appends one JSON line per label, recording and
  thread count (date, commit, minimum times, launches, pose), to follow the
  backends from one change to the next.

`--strategy <odometry|pose_graph>` overrides `slam.strategy`. With
`pose_graph` each run ends with `SlamPipeline::finish()` (the open keyframe
joins the graph, the worker drains) and more lines follow the usual ones:
the back end's counters (keyframes, odometry edges, loop attempts, accepted and
rejected, optimizations, map re-sends, back-end time) with the range of the
accepted loops (keyframe gap, RMS, matched fraction), a `map:` line with the
duplication of the published map (its points, the distinct map-frame voxels of
`mini_map_voxel_size` they fill, their ratio, about 1 when each voxel is
published once, and the points held in the keyframes' views, which overlap),
then the final pose in the
odom frame (it must equal the odometry run's to the last digit) and in the map
frame, the distance between the two and each one's distance from the origin.
`--save-map` then writes the keyframe-rendered map of the final result, last
keyframe included. Use `test/integration/mslam_pose_graph.jsonc`, which is the
integration config with `strategy` and a synchronous `pose_graph` block; with
`"synchronous": false` the worker thread runs and the result is the same.

```bash
build/gcc/test/bench_slam test/integration/mslam_pose_graph.jsonc test/data/living_room_garden.pbscan 1
```

## Sensor link status (`GetStatus`)

`SlamService.GetStatus` (`grpc/slam.proto`) reports whether the SLAM server is
connected to its sensor server: `SENSOR_LINK_STREAMING` (connected, LiDAR and
IMU samples arriving), `SENSOR_LINK_CONNECTED_IDLE` (connected, samples not
arriving, for example a paused `playback_publisher`), `SENSOR_LINK_DISCONNECTED`,
`SENSOR_LINK_NOT_APPLICABLE` (`remote_scanner` is `"local"`) and
`SENSOR_LINK_UNKNOWN`, with the sensor address and the age in milliseconds of
the last scan and IMU sample (-1 if none yet). The Python viewer shows it as a
second LED below the connection LED (see `client/README.md`).

It also reports the estimator: `strategy` (`SLAM_STRATEGY_ODOMETRY` or
`SLAM_STRATEGY_POSE_GRAPH`) and, for the pose graph, `graph_version` (increases
with every result of the back end), `keyframes` and `loop_closures` (accepted
loop edges); all 0 for odometry. The viewer picks `StreamKeyframes` instead of
`StreamMap` when the strategy is `POSE_GRAPH`, and shows the two counts below
the LEDs.

## Map stream (`StreamMap`)

`SlamService.StreamMap` (`grpc/slam.proto`) streams the map as it grows, with a
per-stream cursor like a Kafka offset. `GetMap` and `GetMapIncrements` still
exist but are deprecated: `GetMap` sends the whole map in one message (over
gRPC's default 4 MiB client limit at about 300,000 points) and
`GetMapIncrements` skips increments that arrive while it is sending.

- An **offset** is an index into the history of map increments (one per
  processed scan), not a point count. Offsets start at 0 in each **epoch**.
- A `MapStreamChunk` carries the increments `[first_offset, next_offset)`, in
  order, and the `epoch` they belong to. Every stream sends each increment from
  its cursor on, in order, with no gap and no duplicate, however slow the
  client is; a slow client lags without losing data or affecting other streams
  or the SLAM thread.
- Consecutive increments are concatenated into one chunk of up to
  `max_points_per_chunk` points (0: server default 65,536; values above 200,000
  are lowered to 200,000). An increment larger than the bound is sent alone.
  Increments without points are not sent on their own but still advance the
  offsets, so a chunk's range may cover more increments than it carries points
  for; trailing empty increments are reported by the next chunk.
- The first message of every stream is sent at once: an empty chunk
  (`first_offset == next_offset ==` the stream's start) with the current epoch.
- **Resume**: send the `epoch` and the last `next_offset` you applied. If the
  epoch is the server's current one the stream continues from that offset (an
  offset past the end is clamped to the end); otherwise `start_offset` is
  ignored and the stream starts at 0. Epochs are never 0, differ between server
  processes and change on every SLAM reset.
- **Reset**: every open stream announces it at once with an empty chunk carrying
  the new epoch and offsets 0, then continues from 0. A client that sees a chunk
  with an epoch other than the one it holds discards its local map.

On the server each stream has its own handler thread that copies chunk pointers
(at most 1024 per lock acquisition) out of `map_history_`, then concatenates,
converts and writes without holding the lock. The SLAM thread's calls stay O(1)
and never wait on a client.

## Keyframe stream (`StreamKeyframes`)

With `slam.strategy` `pose_graph`, `SlamService.StreamKeyframes`
(`grpc/slam.proto`) streams the map as keyframes instead of points of one
growing cloud, so a loop closure re-sends poses, not points. With `odometry` it
announces its epoch and stays silent. The contract follows `StreamMap`:

- A **keyframe** has a consecutive `id` from 0 in each **epoch**, a pose in the
  map frame and its mini map in the keyframe's **own** frame (place it with the
  pose). A mini map holds only the voxels (`mini_map_voxel_size`, decided in the
  odom frame) that no earlier keyframe owns, so the union of the keyframes is a
  single layer: each voxel once. Keyframe 0 is the origin keyframe, the seed
  scans at the identity pose, and is sent before the first registered scan. It is sent once, as `KeyframeAdded{epoch, id, pose, cloud}`, in order
  from the stream's cursor on, with no gap and no duplicate.
- **Pose encoding**: `PoseQuat` = `x y z` plus the unit quaternion `qw qx qy qz`
  (Hamilton, w first). It maps straight to viser's `position` and `wxyz` and has
  no Euler singularity (unlike `Pose3D`).
- `KeyframePoses{epoch, graph_version, [(id, pose)]}` carries the current pose
  of keyframes `[0, n)` and is sent when an optimization moved them, and once
  after a client resumes. A slow client may skip intermediate tables but always
  gets the latest. A table never mentions a keyframe the stream has not sent
  yet.
- The first message of every stream, and again after a reset, is
  `KeyframeEpoch{epoch}`: the client drops every keyframe it holds when the
  epoch differs from its own. Epochs are never 0, differ between server
  processes and change only on a SLAM **reset**. This epoch is **not** the
  `StreamMap` epoch: that one also changes whenever the pipeline re-sends the
  point map after an optimization, while the keyframes stay valid.
- **Resume**: send the `epoch` and the next id you need
  (`KeyframeStreamRequest{epoch, next_keyframe_id}`). If the epoch is the
  server's current one the stream continues from that id (past the end is
  clamped), otherwise it starts at 0.
- The open keyframe (the scans since the last one) is not in the stream, so the
  newest few metres appear when the next keyframe opens. The Python viewer keeps
  showing the live transformed scan, which the server publishes in the map
  frame, for that gap.

The server keeps the keyframe clouds as shared pointers and the latest pose
table behind its own mutex; the SLAM thread's calls are O(1) (the pose table is
converted outside the lock and swapped in), and a handler copies at most 1024
pointers per acquisition and writes outside the lock.
