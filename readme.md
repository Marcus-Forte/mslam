# mslam: Minimal Slam by Marcus

## Running playback

Use the main SLAM binary with a recorded `.pbscan` file:

```bash
./build/default/mslam -c config/mslam.json -f test/data/rotate.pbscan -d 10
```

Arguments:

- `-c <file>` loads the SLAM JSON configuration.
- `-f <file>` replays a recorded sensor file instead of connecting remotely.
- `-d <ms>` sets the playback delay between recording entries when using `-f`.

When playback reaches the end of the recording and no more scans are available,
the process exits cleanly.

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

This executable register two point clouds and publish their results in the GL server (https://github.com/Marcus-Forte/learning-opengl) 
`./register_scans host.docker.internal:50051 target.ply src1.ply src2.ply ...`

Data is available at `test/data/scan*.ply`∏