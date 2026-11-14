# CPU / memory improvements

Investigation of high CPU usage (~75% of one core, ~530 MB RSS) in `mslam`.

## Method

- Replayed `test/data/rooms.pbscan` (821 scans, 9600 pts each, ~20 Hz, IMU enabled) with `-d 0`
  using `config/mslam.jsonc` on the aarch64 dev container.
- Per-stage timings from the existing `Slam timing` log line, plus a `gprof` (`-pg`) build.
- Experiment: disabled `dense_map_->addScan` to measure its cost.

## Findings

Average per-scan cost here is ~6.9 ms against a 48 ms scan period (~14% of a core). The reporter's
machine has 4 cores and is likely slower, so ~75% on one core is plausible. The scan loop is
pure compute and never idles long, so it scales directly with per-scan cost.

| Stage (mean per scan) | Time | Share |
|---|---|---|
| Registration (`ImuRegistration::Align`) | 5.6 ms | ~82% |
| Preprocess (deskew + downsample) | 0.65 ms | ~9% |
| `dense_map_->addScan` | 0.5 ms (spikes to 170 ms) | ~7% |
| `map_->addScan` | 0.03 ms | <1% |

gprof hot spots (inflated by `-pg`, ranking is what matters):
`VoxelHashMap::getClosestNeighbor` ~26%, `VoxelHashMap::addScan` ~25%,
`VoxelHashMap::getClosestNNeighbors` ~15%, numerical Jacobian of the 15-DOF IMU cost ~14%,
`std::hash<Voxel3>` (135M calls) ~4%.

Memory: peak RSS 613 MB in the replay. Disabling the dense map drops it to **92 MB**. CPU only
drops ~3% because `getClosestNeighbor` dominates.

## Improvements (ordered by expected payoff)

### 1. Remove or gate the unused dense map (memory, latency spikes)
[Slam.cc](src/slam/Slam.cc) adds every scan to `dense_map_` (0.01 m voxels, 10 pts/voxel), but its only
consumer (`server.updateMapIncrement(dense_map_increment)`) is commented out.
- It holds ~520 MB, growing without bound. It also stores every point twice (`map_` + `map_rep_`).
- `map_rep_` is a growing `std::vector`, so reallocation produces the 170 ms stalls (`denseAddScan`),
  which cause dropped scans in live mode.
- Fix: make it opt-in via config (e.g. `"dense_map": false`), or delete it.

### 2. Make scan matching cheaper: analytical Jacobian for the IMU path
[ImuRegistration.cc](src/slam/registration/ImuRegistration.cc) uses `NumericalCostCentral` over **15** DOFs for the
point-to-plane cost, which only depends on the 6 pose DOFs. That means 30 residual evaluations
per point (each with an `se3Exp`, so sin/cos) when 0 are needed for velocity/biases, and a
per-element object copy/destructor (`~NumericalCostCentral` shows up at 4%).
- Write the point-to-plane Jacobian analytically (6 columns; `J = [n^T R-term]`), as is already done
  for `Point3Distance` in [PointDistance.hh](include/slam/registration/PointDistance.hh).
- Or at least split into a 6-DOF cost with a `NumericalCostForwardEuler`/analytic Jacobian, and let only the IMU
  factor span all 15 DOFs (it is a single residual, so numerical diff there is cheap).
- Expected: Hessian assembly (~1.5 s of the ~5.6 s total in the replay) drops by 3-5x.

### 3. Faster nearest-neighbour search in `VoxelHashMap`
`getClosestNeighbor` does 27 robin_map lookups per query (4.8M queries, 135M hash calls), repeated
`reg_iterations` times per scan.
- Cache the voxel -> bucket pointer for the 27 neighbours per distinct scan voxel, or search the
  home voxel first and early-out when the best distance is already below the distance to the
  voxel boundary.
- Store `Voxel3` as three packed ints/`int64` key and use a better hash (current hash is
  `x*73856093 ^ y*19349669 ^ z*83492791` on `Eigen::Vector3i` reinterpret-cast, which clusters
  in the low bits for robin_map's power-of-two mask).
- Store points in a flat array of `{float x,y,z}` per bucket (fixed `max_points_per_voxel`
  capacity) instead of a `std::vector<Point>` per voxel to avoid pointer chasing / per-bucket heap
  allocations; `Point` also carries intensity, which the search does not need.
- With `resolution=0.1` and `max_correspondence_distance=0.2`, 3x3x3 only guarantees 0.1 m;
  a 0.2 voxel (or fewer lookups with a larger voxel) is both cheaper and more correct.

### 4. Normal estimation cost
`getClosestNNeighbors` + 3x3 eigen-decomposition runs per unique map point (1.4M calls, ~18%).
- Cache normals in the map (compute lazily per voxel, invalidate when the bucket changes) rather
  than per `Align()` call in a `std::unordered_map<float-key>` that is rebuilt each scan.
- Use the voxel's points directly (`max_points_per_voxel=3` gives a poor plane fit anyway); a
  point-to-point or small-covariance (GICP-like) cost avoids the neighbour sort entirely.
- `MapPointKeyHash` XORs float hashes; use a voxel/bucket index key instead.

### 5. Bound the map (CPU grows with map size, memory is unbounded)
`map_` and `map_rep_` are never pruned. Lookups slow down as the map grows and `GetMap` copies it
all. Prune voxels farther than a radius from the current pose (KISS-ICP style local map) and keep
`map_rep_` out of the hot path (build lazily for `GetMap`, or drop it).

### 6. Preprocessing
- `deskew` runs `se3Exp` (sin/cos) for each of the 9600 raw points, then 88% are discarded by the
  voxel filter. Downsample first, or interpolate the twist (e.g. apply `se3Exp` for ~16 time
  buckets and slerp/lerp) instead of per point.
- `downsample()` allocates a fresh `VoxelHashMap` (robin_map + vectors) per scan. Reuse a
  persistent filter object and `clear()` it.
- `removePointsNearCenter` / `filterByIntensity` each copy the full scan; fuse into one pass.
- `downsampleToCentroids` uses `std::map<std::array<int,3>,...>`; switch to a hash map.

### 7. Reduce iteration work
- `reg_iterations=3` x `optimizer_iterations=3` always runs fully when the early-out counter is
  `> 3` hits (`k_maxSmallDeltaHits`), which can never trigger with only 3 outer iterations.
  Use a convergence check on `|delta|` (rotation/translation thresholds) to exit after the first
  iteration when the IMU prior is good.
- Subsample correspondences (e.g. every 2nd-3rd point) for the later iterations.

### 8. Per-IMU-sample overhead (live mode)
- `server.updatePose(getPose())` is called for every IMU sample (~200 Hz): it takes a mutex and
  `notify_all()`s, and each connected viewer gets a serialized gRPC message per sample. Publish
  at a fixed rate (30-60 Hz) or on scan only.
- `Predict()` calls `logState(...)` at **INFO** level for every IMU sample (~200 lines/s even with
  `log_level: info`). Change to DEBUG. With the default `log_level: debug` in
  [mslam.jsonc](config/mslam.jsonc), ~13k lines/s are written through `std::cout` (unbuffered with
  `sync_with_stdio`), which costs ~5% here and more on a tty/docker log driver. Default to `info`
  and use `'\n'`/batched output.
- The main loop polls with `sleep_for(1ms)` when no scan is ready. Use a condition variable
  signalled from the scan callback.
- IMU is drained only when a scan arrives (queue capped at 1000), so bursts of ~10 samples are
  processed back-to-back; fine, but keep the per-sample cost small (preintegration + a
  `toAffine` per sample is currently recomputed in `toGravityCompensatedWorldAcceleration`).

### 9. Build / platform
- The `gcc` preset on aarch64 compiles with generic `-march=armv8-a` (see `/opt/toolchain/gcc.cmake`),
  so on a Pi 5 the `pi5` preset (`-mcpu`/`-march=cortex-a76`) should be used; verify the running binary
  came from that preset.
- Build with `-O3 -DNDEBUG` (already Release) and consider LTO / `-ffast-math` limited to the
  registration TU.

### 10. Correctness issues spotted along the way (thread safety)
- `SlamServer::GetMap` reads `map_->getPointCloudRepresentation()` while the SLAM thread appends to
  it (data race / possible crash). Guard with a mutex or serve a snapshot.
- `ConsoleLogger` writes to `std::cout` from several threads without synchronisation.

## Suggested order

1. Remove/gate dense map (1 line, -520 MB, removes 170 ms stalls).
2. IMU log level fix + default `log_level: info` + rate-limit `updatePose`.
3. Analytical 6-DOF Jacobian for the IMU registration.
4. `VoxelHashMap` lookup/hash/bucket layout + cached normals.
5. Local map pruning and cheaper preprocessing.

Re-measure with `Slam timing` log (`Registration:` and `Total:`) on `test/data/rooms.pbscan`
(baseline: 6.9 ms/scan mean, 613 MB peak RSS, 5.6 s CPU for 821 scans).

---

## Follow-up: bounded map landed, remaining per-scan cost (2026-10-04)

Items 1, 5 and 10 above are now implemented:
- `VoxelHashMap::prune(center, max_range, max_voxels)` bounds the registration map, evicting voxels
  farthest from the current pose (10% headroom). Called from `Slam::pruneMap()` after every scan.
- `getPointCloudRepresentation()` is built lazily instead of being appended on every insert, so
  insertion no longer reallocates a growing flat vector (removes the 170 ms stalls).
- `dense_map_` is gated behind `map.dense_map` (default off), removing ~0.5 GB and its stalls.
- `GetMap` now serves an increment history accumulated in `SlamServer`, decoupled from the bounded
  registration map and guarded by a mutex (fixes the read/write race). The full map is still served.
- New config: `map.max_voxels` (300000), `map.max_range` (0 = off), `map.dense_map` (false).

### Where `Slam::Update` time goes (live remote Mid360, 1295 scans, debug logging)

| Component (per scan) | mean | p50 | p90 | p99 | max |
|---|---|---|---|---|---|
| `Slam::Update` | 20.9 ms | 18.0 | 31.1 | 67.4 | 178.0 |
| KNN (`CorrespondenceFinder`, 3 searches) | 8.8 ms | 7.2 | 13.8 | — | 58.7 |
| remainder (normal estimation + LM optimizer) | 12.2 ms | 10.7 | 17.6 | — | 155.4 |

- A 38 ms update is only ~p92, not an outlier: ~10-20 ms KNN + ~15-25 ms of normal estimation and
  optimizer. With `max_correspondence_distance: 0.5` almost every one of ~3000 downsampled points
  produces a correspondence, and `NormalEstimator` (constructed per `Align`) recomputes a 5-NN lookup
  **and a 3x3 eigendecomposition for every correspondence on every scan**. That is the bulk of the
  ~12 ms remainder and is item 4 above (cache normals per voxel, invalidate on bucket change).
- KNN here is ~1 us/point vs ~0.5 us in the `living_room` replay because the map sits near the new
  300k-voxel cap. Lowering `max_voxels` (or setting `max_range`) trades map extent for lookup cost.

### The >100 ms outliers are wall-clock stalls, not map lookups

Traced the 178 ms scan: 120 ms was spent inside a single LM `computeLinearSystem` call, whose own
distribution is p50 0.12 ms / p99 0.33 ms. That function is a fixed numeric loop over the
correspondences with no I/O or allocation, and the KNN searches in the same scans were also inflated
(up to 54 ms/scan). `steady_clock` counts descheduled time, so the thread was starved / preempted;
this is not a growth-with-map-size effect.

Aggravators observed: `log_level: debug` plus `| tee` (large synchronous I/O), the unbounded
`map_history_` memory used for `GetMap` (RSS grew past 1 GB), and CPU contention from other
processes on the box.

### Next

1. `log_level: info` (keeps the `Slam::* elapsed` lines) and write to a file instead of `| tee`.
2. Per-voxel normal cache (item 4) to remove the ~12 ms remainder.
3. Consider `reg_iterations` reduction / early convergence and correspondence subsampling (item 7).
4. If map extent matters more than lookup cost, tune `max_voxels` / `max_range`.

---

## Follow-up: deterministic benchmark and hot-path work (2026-11-14)

Measured with `bench_slam` (offline, synchronous, deterministic replay through `SlamPipeline`, logging
off, `test/integration/mslam.jsonc`), see the readme. Machine: the dev container (aarch64, GCC 16.2,
Release `-O3`, `-mcpu=native`), noisy host. Numbers are the **minimum of 5 runs**, old and new binaries
interleaved. "Baseline" is the code before this round, timed with the same stage boundaries.

| Recording (scans) | Stage | Baseline (ms) | Final (ms) |
|---|---|---|---|
| living_room_garden (1129) | total | 4932 | 2263 (-54%) |
| | preprocess | 595 | 313 |
| | register (`Slam::Update`) | 4222 | 1855 |
| | map insert | 21 | 17 |
| | IMU predict (all samples) | 10.3 | 8.9 |
| rooms (831) | total | 2401 | 1204 (-50%) |
| | preprocess | 274 | 173 |
| | register | 2058 | 967 |
| | map insert | 12.7 | 11.4 |
| | IMU predict | 7.0 | 6.2 |

### What paid off (living_room_garden, `Slam::Update` split measured with temporary timers)

Baseline: KNN 1.66 s, normal estimation + constraint assembly 1.9 s, LM optimize ~0.43 s (of 4.1 s).

- **P2 + P3 (map layout, KNN, normals): the big win.** Pooled voxel storage, inline mixing hash,
  home-voxel-first search with voxel lower bounds, allocation-free KNN into a stack array, reusable
  `NormalEstimator` with a robin_map cache and closed-form 3x3 eigensolver: KNN 1.66 s -> 0.61 s,
  normals 1.9 s -> 0.92 s. Of that, the closed-form `computeDirect` alone was ~0.12 s (~4.5% of the
  total); an inline hash by itself changed nothing measurable.
- **P4 (preprocessing):** one fused range/voxel/intensity pass over reusable tables, deskew rotation
  increments memoized (bit-identical): preprocess 460 -> 320 ms, and first-seen point order gives the
  registration better locality (register -60 ms).
- **P1 (analytical Jacobian, scalar residual): correct, but no speedup.** The optimizer is only ~9% of
  `Update`; `computeLinearSystem` went from ~445 ms (forward differences) to ~430 ms. Kept because it
  is exact (verified against central differences in `test_point_distance`) and costs nothing, but it
  is not why the numbers above improved.
- **P8 (float `transformCloud`): skipped.** 10.8 M points transform in 10.6 ms (1 ns/point), ~0.4% of
  the run; not worth changing numerics.
- P5/P6/P7 (observer only at debug, per-IMU-sample logs at debug, shared increments) are not visible in
  the offline benchmark (no sink, logging off); they reduce per-sample and per-scan overhead in the
  live system.

### Final poses (x y z roll pitch yaw)

| | living_room_garden | rooms |
|---|---|---|
| baseline | 0.044816669 -0.001382728 0.001677983 -0.003383584 0.002161292 0.485544986 | 0.214577441 0.812122480 -0.100435408 -0.155879427 -0.003174955 0.494384323 |
| final | 0.044355531 -0.000769243 0.001632269 -0.003418023 0.002223298 0.485509109 | 0.138621122 0.860490889 -0.193924006 -0.215396197 0.018934346 0.519457420 |

living_room_garden moves by 0.8 mm (every intermediate build stays within ~1 mm). **rooms moves by
~13 cm**, and that is sensitivity, not a regression: seven intermediate builds that differ only in
floating-point rounding or point ordering (numerical vs analytical Jacobian of the same model, hash
function, normal solver, downsample order) end between 0.17-0.30 m in x and 0.81-0.98 m in y. The
rooms recording has no loop closure and an ill-conditioned drift, so any rounding-level change is
amplified. Use living_room_garden (and `SlamIntegration`) as the correctness gate, and compare rooms
only between builds that must be bit-identical.

---

## IMU / LiDAR time alignment (2026-11-14)

Three defects in how the IMU and the scans were lined up in time, fixed and measured with
`bench_slam` (offline, deterministic, `test/integration/mslam.jsonc`). Machine: the dev container
(aarch64, GCC 16.2, Release, `-mcpu=native`), noisy host. Totals are the minimum of 6 interleaved runs of
old and new binaries.

### How it is measured

`Slam::registrationStats()` (printed by `bench_slam`) counts the scans registered through the IMU path
versus the LiDAR-only fallback, and averages the pose increment the **first** registration round applies
(norm of the se(3) translation / rotation step of the first Levenberg-Marquardt solve): how wrong the
IMU-predicted prior was. It costs a few additions per scan. Built on the code before any fix it reproduces
the numbers measured beforehand with throw-away instrumentation: 1119 IMU scans, 12.754 mm / 1.597 mrad
(living_room_garden); 709 IMU + 112 fallback, 14.970 mm / 3.128 mrad (rooms).

### The fixes

1. **One IMU interval per scan was discarded.** `Slam::Update` (IMU path) cleared the IMU clock, so the
   first sample after every scan only re-seeded it. Now only the preintegrator restarts at the scan; the
   clock is re-seeded where the reset is deliberate (start-up, a gap, `ResetImuPreintegration`, the
   LiDAR-only path). Accurate as described: ~10% of the IMU time was lost (integrated time over
   living_room_garden: 48.13 s before, 53.7 s after, for the same recording).
2. **Deskew targeted the scan start.** `deskewInto` now expresses the points in the sensor frame at the
   scan end, like the other overloads and like the state the scan is combined with. The end rotation is
   computed per IMU interval first and its inverse is the starting orientation of the point sweep, so the
   per-point cost is unchanged (living_room_garden preprocess 312.7 ms vs 314.5 ms). Accurate as described.
3. **IMU samples were grouped with scans by arrival.** New `ImuScanSynchronizer` (used by
   `SlamPipeline::pushImu` / `processSyncedScan`, so `SlamRunner` and `bench_slam` share it): a scan gets
   the samples with `timestamp <= t_end`, the state lands exactly on `t_end` through a split sample, a scan
   is held until the IMU reached `t_end` (live: at most `SlamRunner::kMaxImuWait` = 50 ms, then logged and
   counted). Accurate as described, with one addition found on rooms: the 112 fallbacks were not all
   arrival artefacts, 40 of them are scans inside a real 59.5 s hole of the IMU stream in the recording
   (see below); those still, correctly, register LiDAR-only.

### Results

| | first-round correction | IMU-path / fallback scans | final pose (x y z roll pitch yaw) | total (ms) |
|---|---|---|---|---|
| **living_room_garden** before | 12.754 mm / 1.597 mrad | 1119 / 0 | 0.044355531 -0.000769243 0.001632269 -0.003418023 0.002223298 0.485509109 | 2267 |
| after fixes 1+2 (arrival grouping) | 9.040 mm / 0.700 mrad | 1119 / 0 | 0.044366913 -0.000183768 0.001791816 -0.003345142 0.002097673 0.487693816 | - |
| after 1+2+3 | 8.917 mm / 0.664 mrad | 1119 / 0 | 0.043557395 -0.000103929 0.001146933 -0.003307221 0.002205761 0.487481382 | 2179 |
| **rooms** before | 14.970 mm / 3.128 mrad | 709 / 112 | 0.138621122 0.860490889 -0.193924006 -0.215396197 0.018934346 0.519457420 | 1204 |
| after fixes 1+2 (arrival grouping) | 13.660 mm / 3.055 mrad | 736 / 85 | 0.526204200 1.107554875 -0.137309210 -0.168663286 0.008050113 0.751922227 | - |
| after 1+2+3 | 6.717 mm / 0.478 mrad | 781 / 40 | -0.015196915 -0.116118631 -0.004362015 0.000604909 0.001008117 -0.031340839 | 1081 |

- living_room_garden ends 0.0436 m from the origin (gate 0.1 m); the in-process 16x integration test
  gives 0.043 m on 4 runs. Final heading is unchanged to within 2 mrad.
- Totals: living_room_garden -3.9% (min 2267 -> 2179 ms), rooms -10% (1204 -> 1081 ms). The rooms gain is
  registration (966 -> 833 ms; the cause was not isolated, a better prior is the likely one); its preprocess rose
  174 -> 188 ms because deskew now runs on every scan instead of being skipped for the scans that arrived
  with no IMU sample.
- rooms has no ground truth: its pose is not a gate. It moved from a ~0.9 m / 0.5 rad offset to near the
  origin without any tuning, which is consistent with the recording being a loop, but that is not proven.
- The 40 rooms fallbacks: the recording's IMU stream steps from t = ...685.3 s to ...744.8 s (59.5 s,
  `IMUGAP` seen while debugging) while scans keep arriving for ~2 s of it. Those scans have no IMU data.

### Consistency probe: rooms final pose vs `reg_iterations`

| `reg_iterations` | before (x y yaw) | after (x y yaw) |
|---|---|---|
| 2 | -0.128928890 0.798024612 0.328857488 | -0.015218629 -0.114957373 -0.031438001 |
| 3 | 0.138621122 0.860490889 0.519457420 | -0.015196915 -0.116118631 -0.031340839 |
| 4 | 0.351817603 1.041350424 0.597867470 | -0.015319555 -0.115090200 -0.031420421 |
| spread (max - min) | 0.481 / 0.243 / 0.269 | 0.0001 / 0.0012 / 0.0001 |

The spread across `reg_iterations` shrinks by two to three orders of magnitude. On living_room_garden
the same sweep stays within 1 mm in x and y both before and after, so nothing is lost there.
