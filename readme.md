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

## Docker

Build and run from the repository root:

```bash
docker build -f deployment/Dockerfile -t mslam .
docker run -it --rm -v ./config/:/config/ mslam mslam -c /config/mslam.json
```

If you want a custom configuration, mount your JSON file into the container and
pass that path with `-c`.

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

## Browser viewer

The `viewer/` folder contains a minimal TypeScript browser app for inspecting
local `.ply` files.

```bash
cd viewer
npm install
npm run dev -- --host
```

The viewer supports:

- drag-and-drop or file-picker loading for local `.ply` files
- Z-up camera orientation
- point-size adjustment
- multiple ruler measurements with clear-all
- `W`, `A`, `S`, `D` camera movement

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

`register_scans2d` is a small utility for loading individual `.ply` scans and
running pairwise registration experiments.

```bash
./build/default/register_scans2d scan1.ply scan2.ply
```

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

