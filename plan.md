# mslam – SLAM review & improvement plan

Scope: static review of `src/slam`, `src/map`, `include/slam/registration`, plus a baseline playback of
`living_room.pbscan` (existing `build/gcc/mslam`, no rebuild so sources were not reformatted).

## Baseline (before any change)

| Item | Value |
|---|---|
| Final pose | `pos=[0.147,-0.209,0.007] rot=[0.001,0.006,1.379]` – within the regression tolerance of the reference in `.agents/skills/regression/SKILL.md` |
| Scans processed | 1918 (10 used for init) |
| Per-scan cost (debug logging on) | ~87 ms total: registration ~50 ms, **dense map `addScan` 24–31 ms**, preprocess 3–12 ms |
| Warnings | 58× "Reset IMU preintegration", 1× "Large IMU delta 15.6 s" |

The pipeline works on this dataset, but the issues below are correctness/robustness risks that this dataset does not
exercise (aggressive motion, long runs, restart via gRPC, live sensor).

Note: the regression skill still points at `build/default` and `config/mslam.json`; the real paths are `build/gcc` and
`config/mslam.jsonc` (comments must be stripped for the playback CLI, or the loader must accept jsonc).

---

## P0 – Correctness bugs

1. **`reset()` leaves SLAM in an unrecoverable state** – [Slam.cc](src/slam/Slam.cc) `Slam::reset()` clears the map, but
   `init_scan_count`, `last_pose`, `last_delta`, `last_scan_timestamp_ns` are locals in `run()` and are not reset.
   After a gRPC `Reset` the map is empty, the init phase is skipped, registration finds no correspondences and the
   scan is still appended at the dead-reckoned pose. `dense_map_` is not cleared either.
   *Fix:* move run-loop state into `Slam` members (or a `reset_requested_` flag consumed by `run()`), clear
   `dense_map_`, and restart the init phase.

2. **Data race on reset/start/stop from the gRPC thread** – [SlamServer.cc](src/slam/SlamServer.cc) calls
   `slam_->reset()` from a gRPC worker while `run()` is mutating `state_`, `map_` and `preintegrator_` (no lock).
   *Fix:* make `reset()` only set an atomic request flag that the run loop services between scans.

3. **Orientation integration is wrong** – `Slam::Predict` integrates body-frame gyro rates by adding them to Euler angles
   (`rotation += dt * gyro`). That is only valid for rotation about a single axis from zero attitude and ignores gyro bias
   (`state_.gyro_bias` is not subtracted). Same for the position/velocity dead reckoning, which uses this rotation.
   Existing test `Slam.PredictIntegratesAllAngularAxes` encodes the incorrect behaviour (`pose[3] == 1.5`).
   *Fix:* keep attitude as a rotation matrix/quaternion, integrate `R *= Exp((gyro - bg) * dt)`, derive the scan-matching
   prior from `ImuPreintegrator` (`delta_R_`, `delta_v_`, `delta_p_`) rather than a second, inconsistent integrator,
   and update the tests.

4. **Init phase ignores the pose** – during the first 10 scans `Predict()` still moves `state_`, but scans are added to
   the map untransformed and `last_pose` stays identity. The first real scan then starts from a drifted prior and
   `last_delta = last_pose⁻¹ * new_pose` contains the init drift, which feeds deskew (spurious motion on scan 11).
   *Fix:* hold `state_` at identity during init (or transform init scans by the current pose), and initialise
   `last_pose`/`last_delta` from the state after init. Also downsample init scans like normal scans.

5. **Failed registration is still inserted into the map** – when there are no correspondences (or too few) `Align`
   `break`s and returns the prior; `run()` then `addScan`s at that pose. One bad frame permanently corrupts the map.
   *Fix:* return a status/fitness (inlier ratio, RMS, number of constraints); skip map insertion and fall back to the
   motion-model prediction when below a threshold; log + count consecutive failures.

6. **Correspondence search radius mismatch** – `VoxelHashMap` searches only 1 ring (27 voxels, `adjacent_voxels_ = 1`)
   with `map.resolution = 0.1`, but `max_correspondence_distance = 0.2` (2 voxels). Neighbours at 0.1–0.2 m can be missed
   or a non-nearest point returned. `NormalEstimator` (5 neighbours) has the same limitation.
   *Fix:* derive `adjacent_voxels` from `ceil(max_correspondence_distance / resolution)` (or enforce the invariant in
   config validation) and add a unit test.

7. **IMU bias is unobservable in the current factor** – [ImuRegistration.cc](src/slam/registration/ImuRegistration.cc)
   passes `state_i` (including biases) as a constant input; the bias-corrected deltas only depend on `state_i`'s bias, so
   the optimiser cannot change what the residual depends on, and `bg/ba` just random-walk with the weak bias residual.
   Velocity is then overwritten by a finite-difference estimate. Either make state_i a variable in a sliding window /
   fixed-lag smoother (preferred), or drop bias/velocity from the 15-DOF problem and use IMU only as a prior until that
   exists. The `kImuWeight = 0.01` hack hides this.

## P1 – Accuracy / robustness

8. **IMU/LiDAR time alignment** – `run()` drains *all* queued IMU samples before each scan. For live sensors this can
   include samples newer than the scan, and in playback it relies on file ordering. Consume IMU with
   `t ≤ scan_end_time` only, interpolate at the boundaries, and keep the remainder. Also: the first IMU sample after
   every update is dropped (`last_imu_timestamp_ns_.reset()`), losing one interval; `static uint64_t last_imu_time` in `run()`
   causes an unsigned underflow in the debug log on the first sample.

9. **Robust cost / outlier handling** – only a hard distance gate is used. Add a robust kernel (Huber / Geman-McClure) or
   iteratively reweighted LS, an adaptive gate (e.g. KISS-ICP style threshold from recent motion error), and a
   single-scalar point-to-plane residual (currently a 3-vector `n * (n·d)` – triples the residual dimension and cost for
   the same objective).

10. **Normal quality** – `NormalEstimator` accepts any ≥3 neighbours. Reject non-planar neighbourhoods (e.g.
    `λ0/λ1` ratio, minimum neighbour spread) and orient normals consistently; compute normals once when a voxel is
    inserted instead of per scan per correspondence. Replace the weak XOR float hash for the cache.

11. **Degeneracy detection** – corridors/flat floors leave directions unconstrained. Inspect the Hessian eigenvalues
    after the last iteration and constrain those directions with the prior (motion model/IMU) instead of letting the
    optimiser drift. Expose the 6×6 covariance (already on the todo list).

12. **Euler-angle parametrisation** – pose is stored as roll/pitch/yaw (`R = Rx·Ry·Rz`, extraction via
    `asin(R(0,2))`): singular at pitch ±90° and expensive (`toAffine` builds 3 `AngleAxis` per call). Store
    `SE3`/quaternion in `SlamState`; keep Euler only for the gRPC/pose output.

13. **Deskew assumptions** – `deskew()` assumes point index ∝ time and a constant-velocity twist from the previous
    scan delta; after filtering/reordering in the driver this no longer holds. Use per-point timestamps if
    `msensor` provides them; otherwise document the assumption and test it. Also guard `num_points == 1` (division by 0
    in `i / (num_points-1)`), and `delta_t` very small/large.

14. **Gravity / constants duplicated** – `9.80665` is hard-coded in `Slam.cc`, `ImuPreintegration.cc`,
    `ImuRegistration.cc`; the gravity direction is fixed to +Z. Centralise it, and estimate/refine it (and the
    yaw-unobservable alignment) from the init window instead of using only the first IMU sample (noisy; average the
    first N static samples).

15. **Preintegration covariance** – `Q` for the position block adds `na² dt` independently of the propagation through
    `F`, which double counts; verify against a reference (e.g. GTSAM `PreintegratedImuMeasurements`) in a unit test.

## P2 – Map / performance

16. **Dense map is computed but never used** – `dense_map_->addScan` costs 24–31 ms/scan (≈30 % of frame time at
    1 cm / 10 pts per voxel) and grows without bound while the publish call is commented out. Make it opt-in via config.

17. **Unbounded map** – `VoxelHashMap::map_rep_` keeps every point forever and the first 3 points per voxel win
    (no replacement or averaging). Add a local map (drop voxels beyond a radius from the current pose), keep
    `map_rep_` derivable on demand, and consider keyframe-based insertion (only insert when moved/rotated enough) to avoid
    repeated insertion while stationary.

18. **Numerical differentiation** – `PointToPlaneRegistration` uses `NumericalCostForwardEuler`; analytic Jacobians exist
    in `PointDistance.hh` but are Euler-parametrised and unused, i.e. dead code that no longer matches the SE3 plus
    operator. Replace with analytic SE(3) Jacobians (`[−n, (R s × n)]`) – large speed-up and better convergence than forward Euler.
    Remove or fix the unused 2D/3D/point-point models.

19. **KNN loop** – `CorrespondenceFinder` is single-threaded (≈6 ms of ~50 ms registration); parallelise per point
    (also on the todo list), and avoid the per-scan `std::map` + `NormalEstimator` cache allocations. Check
    `downsampleToCentroids` (`std::map`) for the same.

20. **Logging in the hot path** – `config/mslam.jsonc` ships with `log_level: debug`; the 1918-scan replay took 4.5 min at that level (info level not measured);
    guard formatting with level checks and keep the default config at `info`.

## P3 – Features (SLAM, not just odometry)

21. Keyframes + local map management, then **loop closure** (scan-context or ICP against submaps) and pose-graph
    optimisation (the `moptim` backend can be reused). Currently this is LiDAR(-inertial) odometry only, so drift is
    unbounded.
22. Per-scan quality metrics published over gRPC (fitness, inlier ratio, timing) for the viewer/diagnostics.

## Tests to add (none exist today for these)

- `NormalEstimator`: planar patch, collinear points, too few points.
- `PointToPlaneRegistration`: synthetic planes (floor + 2 walls) with known SE3 offset; assert recovery, and that
  degenerate input (single plane) is flagged.
- `ImuPreintegrator`: constant rotation/acceleration closed-form check; bias Jacobian vs finite differences;
  `residual()` ≈ 0 for consistent states.
- `VoxelHashMap`: neighbour search for `max_correspondence_distance > resolution`.
- `Slam`: `reset()` mid-run then re-init; IMU/LiDAR time-ordering; failed-registration does not insert into the map.
- End-to-end: scripted regression (`living_room.pbscan` + `eratolaan.pbscan`) in CTest with pose tolerances, replacing the
  manual skill steps (and fix its stale paths).

## Suggested order

1. P0 #1, #2, #4, #5 (restart/init/robustness; small, localised changes) → rerun regression.
2. P2 #16, #20 (quick wins in runtime) and P0 #6 (search radius) → rerun regression, compare timings.
3. P0 #3 + P1 #8, #12 (attitude representation & IMU timing) together, updating `test_slam.cc`.
4. P1 #9–#11 (robust cost, normals, degeneracy) + P2 #18 analytic Jacobian.
5. P0 #7 / sliding-window IMU, then P3 loop closure.

After each step: `cmake --build --preset build-gcc`, `ctest --test-dir build/gcc`, then the regression replay
(final pose must stay within tolerance of `pos=[0.150,-0.205,0.012] rot=[0.001,0.007,1.379]`).
