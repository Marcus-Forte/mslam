# mslam: Minimal Slam by Marcus

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

## TODO
- Have sensors store timestamp at time of sampling.
- Finish voxel hashing
- Implement point-to-line error model
- Implement acceleration and velocity model

## Registering individual scans

`register_scans` is a small utility for loading individual `.ply` scans and
running pairwise registration experiments.

```bash
./build/default/register_scans scan1.ply scan2.ply
```