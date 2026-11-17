# msim

MuJoCo-based Livox Mid360 + IMU simulator that acts as a remote sensor server for mslam, like
`playback_publisher`: set `remote_scanner` in the mslam config to `host:port` (default port 50051).
Draw rooms in YAML, fly the sensor with the keyboard or a scripted path, get ground truth for drift measurements.

Python 3.13+, uv project. Everything runs on the host.

## Setup and tests

    cd simulator
    uv sync                 # mujoco, numpy, grpcio, protobuf, pyyaml (+ pytest, grpcio-tools in the dev group)
    uv run pytest           # 87 tests, ~25 s, includes the end-to-end gRPC test

The gRPC stubs are generated from `third_party/msensor/proto` on first use (`uv run msim gen-proto` to force); they are not checked in.

## Run

Headless, scripted, deterministic (waits for both streams to have a client, then runs flat out and loses nothing):

    uv run msim serve --scene scenes/two_rooms.yaml --headless --trajectory trajectories/two_rooms_loop.yaml \
        --unpaced --gt-log gt.csv --exit-when-done --port 50051
    uv run msim check localhost:50051          # in another terminal: rates, counts, timestamps, sequence gaps

Real time (what mslam sees on a live sensor; slow clients lose the OLDEST samples, as the real server does): drop `--unpaced`.

Viewer with keyboard flight. The MuJoCo passive viewer needs `mjpython` on macOS:

    uv run mjpython -m msim serve --scene scenes/two_rooms.yaml

If `mjpython` fails with `libpython3.13.dylib` not loaded (uv's standalone Python), link it once:
`ln -s <uv python dir>/lib/libpython3.13.dylib .venv/lib/` (the path is in the error message).

Keys (WASD is not used: MuJoCo's viewer binds every letter to a display toggle, e.g. W wireframe, S shadows, R reflections. S, W, R, L, K and G cannot be undone from Python, so the controls avoid them; the other display toggles are restored every frame):

| Key | Action |
|---|---|
| Up / Down | forward / back |
| Z / X | strafe left / right |
| E / Q | up / down |
| Left / Right | turn left / right (yaw) |
| Y / H | look up / down (pitch) |
| U / O | roll left / right |
| N (hold) | level the sensor: z axis back to world up (roll and pitch to 0), heading kept |
| B | toggle translation mode `level` / `body` (printed on the console) |
| Space | brake (release all keys) |
| Enter | reset to the start pose and attitude |
| Tab / Shift+Tab | the viewer's own show/hide of the left / right panel (reserved, not used by msim) |

The left and right MuJoCo panels are hidden at start; `--show-ui` shows them.

MuJoCo forwards key presses and repeats but not releases, so a key counts as held for 0.45 s after the first press and 0.18 s after each repeat. Motion is jerk-limited (2 m/s, 2 m/s^2, 8 m/s^3 and 1.2 / 0.8 / 0.8 rad/s for yaw / pitch / roll by default) and the sensor slides along walls (0.25 m sphere). Mouse look is not implemented. The green dots are the last scan (400 points, not deskewed). `--trajectory` in viewer mode just plays the path.
The viewer was only smoke-tested (it starts and runs for 10 to 15 s without errors); the keys could not be pressed in that test and have not been driven by hand. They are covered by unit tests.

### Attitude model and translation modes

The interactive sensor has the full 3D attitude: no pitch limit, any orientation including inverted. The controller keeps the rotation matrix `R` (body to world) and three jerk-limited body-frame angular-rate channels (about x = roll, y = pitch, z = yaw). Each 1 ms tick updates the rates first and then `R <- R Exp(w dt)`; the same `w` goes to the gyro, so integrating the gyro reproduces the attitude (test: below 1e-6 rad after a 12 s mixed manoeuvre).

* `level` mode (default): forward/left follow the horizontal heading of the body x axis, up/down is world z. When x is within 1e-3 of vertical the last heading is kept. Past 90 degrees of pitch the horizontal projection of x reverses, so forward then goes the other way.
* `body` mode (press B): forward/left/up are along the body axes, like flying.
* Collisions slide in both: the blocked world velocity component is removed and the channel velocities are re-derived from what is left (`M^T v`, `M` the orthonormal channel-to-world matrix), accelerations cleared.
* The level key commands body rates `k * log(R^T Rz(heading))` (k = 2 /s) through the same jerk/acceleration limits: smooth and physical. From roll 40 / pitch -50 the z axis component reached above 0.999 in 1.5 s with a 1 degree heading change; it also works from exactly upside down.
* Scripted trajectories use Euler angles `R = Rz(yaw) Ry(-pitch_up) Rx(roll)`: pitch positive = nose up, roll positive = right-hand rotation about x (left side up). Waypoints take `yaw_deg`, `pitch_deg`, `roll_deg` (see `trajectories/tumble.yaml`). With roll 0 the output is bit-identical to the version without roll (tested against hashes captured before the change).
* The ground-truth CSV roll/pitch/yaw columns use that ZYX convention; the quaternion is the primary attitude. `msim compare` works on rotation matrices and handles mslam's Rx Ry Rz Euler angles; it only assumes the first ground-truth pose is level (it removes the initial yaw and position, like mslam's gravity alignment).

mslam side: copy a config, set `"remote_scanner": "host:50051"` (`host.docker.internal:50051` from a container) and keep `preprocessor.points_per_second` at 200000.

Other commands: `gen-scene scene.yaml -o scene.xml --check`, `gen-trajectory rect --center 0,0 --size 3,2 --z 1 -o t.yaml`, `compare gt.csv bench_trajectory.csv`.
`serve --record x.pbscan` writes an msensor recording (same framing as `ScanRecorder`), replayable with `bench_slam` / `playback_publisher`.
Noise (all off by default, seeded by `--seed`): `--range-noise`, `--intensity-noise` (relative, e.g. 0.1), `--imu-accel-noise`, `--imu-gyro-noise`, `--imu-accel-bias-walk`, `--imu-gyro-bias-walk`, `--imu-accel-bias`, `--imu-gyro-bias`.

## What it emulates (from the msensor / mslam sources)

* Streams `sensors.LidarService/getLidarScan` and `sensors.ImuService/getImuData` on one port. Per-client bounded queue, drop-oldest; `max_queued_samples` 0 = default (IMU 200, LiDAR 1), capped at 4000 / 64; max 8 clients per stream; UNAVAILABLE for a disabled stream. mslam asks for 1000 IMU and 5 scans.
* One nanosecond clock for both streams. Scan: 9600 points, 5 us apart, header = first point, one scan every 48 ms. IMU: 200 Hz, phase offset 1.3 ms from the scans, so a scan end falls between samples as with the real sensor. Default start at 1e12 ns (`--start-ns`).
* Every point is cast from the sensor pose at its own time (1 kHz pose history, linear position, nlerp attitude), expressed in the sensor frame at that time, so the cloud needs deskewing. x forward, y left, z up, metres, float32.
* Misses keep the point count: they are `(0,0,0)` with intensity 0, exactly what the real Mid360 stream contains (checked in `living_room_garden.pbscan`: 3 to 16 % zeros); mslam's `min_distance_to_center` / `min_intensity` remove them. Minimum range 0.1 m; the maximum depends on the target (below).
* Intensity is Livox's calibrated reflectivity (`src/msim/reflectivity.py`): `150 * reflectance * cos(incidence)` for diffuse surfaces (0..150), `151 + 104 * cos(incidence)` for retro-reflectors (151..255). A return is detected up to `40 m * (s / 0.1)^0.269` with `s = reflectance * cos(incidence)`, the power law through the datasheet's 40 m at 10 % and 70 m at 80 %; beyond it the point is a miss. Indoors this only drops grazing returns.
* Surface reflectance is set per kind in the scene (`reflectivity: {wall: 0.4, floor: 0.2, ceiling: 0.45, obstacle: 0.3}`, the defaults) and per wall, room or box (`reflectivity: 0.1`, `retroreflective: true`); format at the top of `src/msim/scene.py`. The defaults are calibrated so that `two_rooms_loop` has the median intensity of `living_room_garden` (49 against 48); 53 % of its points are below 50, as in the recording, so the integration config's `min_intensity: 50` drops about half of them, as it does on real data.
* IMU: specific force `R^T (a - g)` in g (a level static sensor reads +1 g on z, as mslam's gravity alignment expects), gyro in rad/s in the sensor frame, from the same trajectory (analytic velocity/acceleration; the game controller defines acceleration as the velocity difference per tick, so integrating the IMU reproduces the position).
* IMU sequence numbers count from simulation start whether or not a client is connected (as the Mid360 IMU); a scan only gets a number if it was generated for a consumer.
* Back-pressure: paced runs drop (real behaviour). `--unpaced` uses `block` (the simulator waits for the client), which the real server does not have; it makes runs lossless and reproducible.

Scan pattern: replayed from the real sensor. The Mid360 has 4 beams that spin fast in azimuth (1.30 deg per 4-point group, about 181 rev/s, 8.7 turns per 48 ms scan) while the elevation nods slowly between about -7 and 52 deg with a period of about 102 ms (2.1 scans, not locked to the scans). So a scan covers a band of elevation that changes from scan to scan: some see only -7..18 deg, others the whole range. `msim gen-pattern [recording.pbscan]` measures this from a recording: the sensor-frame direction of every return, with the misses (21 % in `living_room_garden`) interpolated per beam over time (median error 0.02 deg for gaps under 0.2 ms, 0.8 deg for 2 ms gaps; 86 % of the misses are in runs of 2 ms or less). It writes `out/mid360_pattern.npz` (git-ignored, 87 MB, 1129 scans), and simulated scan k uses recorded scan k, cycling every 54 s.
`--pattern` takes one of:

* `measured` (default): that measured table, built from `../test/data/living_room_garden.pbscan` on first use (2 s). Without the table or the recording (Git LFS not pulled) the simulator stops with an error; it does not fall back silently.
* `rosette`: the synthetic approximation.
* a file: a pattern written by `msim gen-pattern other.pbscan -o other.npz`, or a `.npy` table.

The rosette (6 lasers, 60 rev/s, sinusoidal elevation) covers the whole field of view every scan and revisits the same directions, which the real sensor does not:

| per scan, scans 300..399 | real (returns) | recorded pattern | rosette |
|---|---|---|---|
| elevation span p10 / median / p90 | 28 / 43 / 53 deg | 29 / 44 / 54 deg | 59 deg always |
| 6x4 deg cells covered | 69 % | 70 % | 92 % |
| 1x1 deg cells after 1 / 2 / 5 / 10 scans | 32 / 54 / 83 / 92 % | 37 / 61 / 90 / 97 % | 29 / 34 / 47 / 66 % |

(The real column counts returns only, hence slightly lower.) `--pattern table.npy` still takes an (M,3) / (M,2) table; scan k uses rows k*9600...

## Files

* `scenes/*.yaml`: floor plans (format documented at the top of `src/msim/scene.py`). `single_room.yaml` is hand-checkable (inner faces at 1.9 m, floor 1 m below, ceiling 1.8 m above the start). `two_rooms.yaml` has two rooms, a doorway and six obstacles.
* `trajectories/*.yaml`: quintic-spline waypoint paths that start and end at rest at the same pose (loops).
* `src/msim/`: `scene`, `raycast` (numpy slab test = fast path; `mj_ray` caster = reference), `pattern`, `motion`, `history`, `sensors`, `engine`, `streams` (gRPC), `recorder`, `check`, `compare`, `viewer`, `app`, `cli`.

## Ground truth

`--gt-log` writes `timestamp_ns,seq,x,y,z,qw,qx,qy,qz,roll,pitch,yaw`: one row per scan at the scan's LAST point time (where mslam registers it), pose of the sensor in the scene frame (z up), quaternion (w,x,y,z), roll/pitch/yaw in rad with R = Rz Ry Rx. mslam's world frame starts at the first sensor pose with yaw 0 and uses R = Rx Ry Rz; `msim compare gt.csv est.csv` (est = `bench_slam --trajectory`) converts and matches on timestamp + 47.995 ms.

## Measured (Apple M3, Python 3.13, mujoco 4.0.1, numpy, two_rooms: 17 boxes)

* Ray casting, 9600 per-point-origin rays: numpy `BoxCaster` 6.1 ms/scan (1.6 M rays/s); looping `mujoco.mj_ray` 20.3 ms/scan (0.47 M rays/s). Both are faster than the 48 ms scan period, so real time works with either (`--caster mujoco`); `mj_multiRay` was not used because it shares one origin. They agree to 1e-6 m on 2900 random rays (test).
* End to end through gRPC with the check client, unpaced: 85 scans/s with the IMU stream (about 4x real time). Real-time run: 200 Hz IMU and 20.8 Hz scans, no gaps (test). A viewer run showed a 0.56 s lag warning before the point drawing was limited to one redraw per scan; after that none in 15 s.

## Known limitations

* Not tested against a live mslam process over gRPC. One offline check was done (with `bench_slam` built in the dev container; the simulator itself runs and is tested on the host only): `two_rooms.pbscan` written by `--record` (829 scans, 39.8 s, 25.6 m path) replayed through `bench_slam` with `test/integration/mslam.jsonc`: 829 poses matched to ground truth, position error mean 16.3 mm, max 27.3 mm, final 0.3 mm; rotation error mean 0.41 mrad, max 1.33 mrad; all 819 registered scans on the IMU path. This is a noise-free, clean-IMU simulation in a box world, so it says the conventions (time, frame, gravity, units) agree, not how mslam behaves on real data.
* Sensor frame = IMU frame (no lever arm or extrinsics); walls are boxes (no curved geometry); box scenes only; one reflectance per surface (no textures). The real recordings have a long tail of very weak returns (24 % below 20 in `living_room_garden`, mostly closer than 1 m, probably the operator) that the simulator does not produce (2.5 %).
* Pose history is 8 s; the engine must not be held up longer than that by a client in `block` mode.
* Keyboard release detection is approximate (see above); no mouse look; no pause key.
