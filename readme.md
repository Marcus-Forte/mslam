# mslam: Minimal Slam by Marcus

## Configuration

Check `config/mslam.json` for the current SLAM configuration. The main SLAM binary loads this JSON file at startup. The configuration includes: 
- which sensors to use (IMU, LiDAR, camera)
- sensor source (local or remote) (currently only remote supported)
- logging level
- map type (voxel or kdtree)
- remote scanner address for live SLAM
- scan preprocessing parameters (downsampling, deskewing)
- map resolution

## Docker

Build and run from the repository root:

```bash
docker build -f docker/Dockerfile -t mslam .
docker run -it --rm -v ./config/:/config/ mslam mslam -c /config/mslam.json
```

If you want a custom configuration, mount your JSON file into the container and
pass that path with `-c`.

For the Python viewer client.
```bash
docker build -f docker/DockerfileViewer -t mslam-viewer .
docker run --rm -it mslam-viewer --server-addr <address>
```

## Running playback

Use the main SLAM binary with a recorded `.pbscan` file:

```bash
./build/default/mslam -c config/mslam.json -f test/data/rotate.pbscan -d 10
```

To export the final clouds at the end of playback:

```bash
./build/default/mslam -c config/mslam.json -f room.pbscan -d 1 -o out/final_cloud
```

Arguments:

- `-c <file>` loads the SLAM JSON configuration.
- `-f <file>` replays a recorded sensor file instead of connecting remotely.
- `-d <ms>` sets the playback delay between recording entries when using `-f`.
- `-o <path>` writes two PLY files on exit: `<path>_voxel_hash.ply` and `<path>_transformed_scans.ply`.

When playback reaches the end of the recording and no more scans are available,
the process exits cleanly.

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

3D registration supports both point-to-point and point-to-plane metrics. The
caller selects the metric through `Registration::Align3D(...)`. The current
main SLAM path uses the point-to-plane metric.

All currently implemented point metrics provide analytical Jacobians and are
used with the analytical optimizer path.

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

The project compiles with `-march=native`, which allows the compiler to emit
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