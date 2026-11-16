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

---

## Codebase Assessment: `mslam`, `moptim`, and `msensor` (2026-11-16)

Static review of `repos/mslam`, `repos/moptim`, and `repos/msensor` covering performance optimizations, code structure/correctness improvements, and architectural improvements. Items already noted in each repo's `AGENTS.md` / `docs/` are marked *(Known)*; new findings from the code scan are marked *(New)*.

---

### Part 1: `repos/mslam`

#### 1.1 Performance Optimizations

1. **Hoist `ImuPreintegrationFactor` (and `sqrtInformation()`) out of the ICP loop** *(New)*
   - **Location:** `src/slam/registration/ImuRegistration.cc:73-98`, `src/slam/ImuPreintegration.cc:227-242`
   - **Issue:** Inside `ImuRegistration::Align`, `ImuPreintegrationFactor(preintegrator, kImuWeight)` is constructed on **every outer registration round**, calling `preintegrator.sqrtInformation()`—which inverts a $15 \times 15$ covariance matrix and computes an $\text{LL}^\top$ Cholesky decomposition each round—even though `preintegrator` and `kImuWeight` are constant across all rounds of `Align()`.
   - **Fix:** Construct `ImuPreintegrationFactor` once before the `for (int i = 0; i < parameters.registration_iterations; ++i)` loop.

2. **Analytical Jacobian for `ImuPreintegrationFactor`** *(Known open hotspot, ~11% of runtime)*
   - **Location:** `include/mslam/slam/registration/ImuPreintegrationFactor.hh:89-104`, `src/slam/registration/ImuRegistration.cc:94-98`
   - **Issue:** `ImuPreintegrationFactor` uses `moptim::NumericalCostCentral` over a 24-DOF tangent space, requiring $2 \times 24 + 1 = 49$ evaluations of the 24D residual per LM linear system. Each evaluation runs `expSO3`, `logSO3` (`Eigen::AngleAxisd`), and a $15 \times 15$ dense matrix-vector product.
   - **Fix:** Implement the analytical $24 \times 24$ `jacobian()` on `ImuPreintegrationFactor` using the bias Jacobians (`J_R_bg_`, `J_v_bg_`, `J_v_ba_`, `J_p_bg_`, `J_p_ba_`) already maintained by `ImuPreintegrator`, and switch to `moptim::AnalyticalCost`.

3. **Sparse/block covariance update in `ImuPreintegrator::integrate` & skip in `DeadReckoning` mode** *(New)*
   - **Location:** `src/slam/ImuPreintegration.cc:121-147`, `src/slam/Slam.cc:134, 175-181`
   - **Issue:** Every IMU sample (~200 Hz) allocates dense $15 \times 15$ matrices `F` and `Q` on the stack and computes `covariance_ = F * covariance_ * F.transpose() + Q`, even though `F` has only six non-identity $3 \times 3$ blocks and `Q` is diagonal. Additionally, when `imu_fusion == ImuFusion::DeadReckoning`, `preintegrator_.integrate()` still propagates the $15 \times 15$ covariance and five $3 \times 3$ bias Jacobians on every IMU sample even though only `preintegrator_.deltaTime()` is ever read.
   - **Fix:** Exploit the block structure of `F` and diagonal structure of `Q`, and skip covariance/Jacobian propagation when in `DeadReckoning` mode.

4. **Cache surface normals in the map instead of per-`Align()`** *(Known open hotspot, ~35% of runtime)*
   - **Location:** `include/mslam/slam/NormalEstimator.hh:21-67`, `src/slam/NormalEstimator.cc:12-30`, `src/map/VoxelHashMap.cc:49-75`
   - **Issue:** `NormalEstimator::reset()` clears `cache_` at the start of every scan's `Align()`. Because map voxels saturate (`max_points_per_voxel = 3..5`), most matched map points and their 5-NN neighborhoods are unchanged from previous scans, yet `NormalEstimator` re-runs a 5-NN search + $3 \times 3$ eigensolve on every scan. It also keys `cache_` by bit-cast float coordinates (`MapPointKey`) rather than pool index.
   - **Fix:** Store normal estimates alongside `pool_` in `VoxelHashMap` (or indexed by pool slot `bucket.offset + i`) and invalidate only when a voxel or its immediate neighborhood receives a new point.

5. **Filter before deskewing & avoid redundant IMU copy/sort in `deskewInto`** *(Known open hotspot + New)*
   - **Location:** `src/slam/Preprocessor.cc:142-157`, `src/slam/Deskew.cc:170-183, 232-263`
   - **Issue:**
     - `Preprocessor::process` runs `deskewInto` on all 9,600 raw points before `VoxelFilter::apply` discards ~89% of them. Range filtering (`min_distance_to_center`) is rotation-invariant ($\|R p\|^2 = \|p\|^2$) and intensity filtering is independent of geometry. For `VoxelHash` (first point per voxel), filtering before deskewing—or carrying point index/timestamp through the filter—reduces deskew work by ~9x.
     - `deskewInto` copies and `std::sort`s `imu_samples` on every scan (`Deskew.cc:170-182`), even though `ImuScanSynchronizer::take` already hands out samples filtered and ordered by timestamp.

6. **Avoid `Transform::rotation()` SVD in `se3Log`** *(Known open issue)*
   - **Location:** `include/mslam/slam/SE3.hh:21`
   - **Issue:** `const Eigen::Matrix<T, 3, 3> R = T_ab.rotation();` runs an SVD polar decomposition to extract the rotation from an `Eigen::Affine3d`, whereas `T_ab.linear()` directly reads the $3 \times 3$ rotation block.

7. **Fix ICP early-exit logic and redundant `resetSource` on the final iteration** *(Known open issue + New)*
   - **Location:** `src/slam/registration/ImuRegistration.cc:33, 110-116`, `src/slam/registration/PointToPlaneRegistration.cc:22, 61-70`, `src/slam/registration/PointToPointRegistration.cc:22, 62-70`
   - **Issue:** `k_maxSmallDeltaHits = 3` with `if (++small_delta_hits > k_maxSmallDeltaHits)` requires 4 hits, so it never triggers when `reg_iterations = 3`. Moreover, on the final iteration `i == reg_iterations - 1` (or when `step` is negligible), `icp_.resetSource(total_T)` still copies and transforms the entire scan cloud (`IcpWorkspace.cc:24-27`) even though `source_` is never read again after the last round.

8. **Batch pose publication in `SlamPipeline::processSyncedScan` and replace polling sleeps in `SlamRunner`** *(Known + New)*
   - **Location:** `src/slam/SlamPipeline.cc:42-66`, `src/slam/SlamRunner.cc:88, 97`, `src/slam/SensorInput.cc:27-68`
   - **Issue:**
     - `processSyncedScan` calls `processImu` in a tight loop for all ~10 queued IMU samples in the scan's window; each `processImu` calls `publishPose()`, locking `pose_mutex_` and calling `pose_cv_.notify_all()` ~10 times in microseconds right before `processScan` publishes the updated pose.
     - `SlamRunner::run` polls with `sleep_for(1ms)` and `sleep_for(500us)` instead of waiting on a condition variable signaled by `SensorInput` callbacks.

9. **Parallelize 1-NN correspondence search (`CorrespondenceFinder::find`)** *(Known open hotspot, ~26% of runtime)*
   - **Location:** `src/slam/CorrespondenceFinder.cc:19-28`
   - **Issue:** `map.getClosestNeighbor(query)` is read-only and independent per scan point, making it straightforward to parallelize across threads or vectorize distance checks within a voxel bucket.

#### 1.2 Code Structure Improvements

1. **Deduplicate registration loops and reuse `LevenbergMarquardt` workspace across scans** *(New)*
   - **Location:** `src/slam/registration/PointToPlaneRegistration.cc:19-74`, `src/slam/registration/PointToPointRegistration.cc:19-75`, `src/slam/registration/ImuRegistration.cc:29-126`
   - **Issue:** All three registration classes duplicate the outer ICP loop, `k_maxSmallDeltaHits`, `first_round_` calculation, and observer attachment. They also instantiate a fresh `moptim::LevenbergMarquardt<double> lm(manifold_)` on the stack on every scan, reallocating internal solver vectors/matrices every call instead of keeping `lm` as a member of the registration class or `IcpWorkspace`.

2. **Switch `PointToPointRegistration` from `NumericalCostForwardEuler` to `AnalyticalCost`** *(New)*
   - **Location:** `src/slam/registration/PointToPointRegistration.cc:55-58`, `include/mslam/slam/registration/PointDistance.hh:117-125`
   - **Issue:** `Point3Distance` already defines an analytical left-increment `jacobian()` in `PointDistance.hh:117-125`, but `PointToPointRegistration` still wraps it in `moptim::NumericalCostForwardEuler`.

3. **Consolidate duplicated SO(3) / SE(3) math** *(New)*
   - **Location:** `include/mslam/slam/SE3.hh`, `src/slam/ImuPreintegration.cc:8-12, 51-80`, `src/slam/Deskew.cc:26-32`, `src/slam/Slam.cc:152-157`, `src/slam/Transform.cc:15-39`
   - **Issue:** Skew-symmetric hat (`hat` vs `skewSymmetric`), SO(3) exponential (`expSO3` vs `rotationIncrement` vs inline `AngleAxisd` in `Slam.cc` vs `se3Exp`), and SO(3) logarithm (`logSO3` vs `se3Log`) are implemented in 4 separate places.

4. **Share strict JSONC parsing/validation helpers between `RegistrationConfig` and `JsonConfig`** *(New)*
   - **Location:** `src/config/JsonConfig.cc:37-184`, `src/config/RegistrationConfig.cc:25-120`
   - **Issue:** `RegistrationConfig.cc` has clean helpers (`parseObjectFile` with `rejectDupKeys`, `rejectUnknownKeys`, `readPositiveFloat`, `readCount`), whereas `JsonConfig::load()` uses raw `file >> root;` without checking if the file opened, does not reject unknown or duplicate keys, and repeats manual `.empty()` checks.

5. **Expose hardcoded estimator constants in `SlamConfiguration`** *(Known + New)*
   - **Location:** `src/slam/registration/ImuRegistration.cc:35` (`kImuWeight = 0.01`), `include/mslam/slam/Slam.hh:100` (`ImuNoiseParams{}`), `include/mslam/slam/registration/ImuPreintegrationFactor.hh:14-18` (`ImuPriorSigma`), `src/main.cc:85` (`setNumAdjacentVoxelSearch(1)`)
   - **Issue:** Tuning parameters and noise models are hardcoded in headers/sources rather than configured or derived from `max_correspondence_distance / resolution` (as `ScanSequenceRegistration.cc:34-40` already does).

#### 1.3 Architectural Improvements

1. **Replace Euler angles in `SlamState` with `Eigen::Matrix3d` or `Eigen::Quaterniond`** *(Known open issue)*
   - **Location:** `include/mslam/common/State.hh:21-27`, `src/slam/Transform.cc:15-45`, `src/slam/Slam.cc:136, 158`
   - **Issue:** `SlamState` stores orientation as Euler angles `(roll, pitch, yaw)`, which is singular at $\pm 90^\circ$ pitch and forces `Slam::Predict` to construct 3 `Eigen::AngleAxisd` objects (`eulerToRotation`) and run `atan2`/`asin` (`rotationToEuler`) on every single IMU sample (~200 Hz). Keeping rotation as `Eigen::Matrix3d` (or `Eigen::Quaterniond`) in `SlamState` and converting to Euler only in `Slam::getPose()` eliminates both the singularity and per-sample trig overhead.

2. **Unify `Slam::Predict` dead-reckoning with `ImuPreintegrator`** *(New)*
   - **Location:** `src/slam/Slam.cc:129-160`
   - **Issue:** `Slam::Predict` integrates the IMU stream twice using slightly different numerical schemes: once in `preintegrator_.integrate` and once in `state_` (forward Euler on position/velocity with Euler-angle round-trips). Predicting `state_` directly from `previous_state_` and `preintegrator_`'s `deltaRotation()`, `deltaVelocity()`, and `deltaPosition()` guarantees the optimization prior and preintegration factor are mathematically consistent.

3. **Fix tilted initialization and multi-sample gravity alignment** *(Known open issue)*
   - **Location:** `src/slam/Slam.cc:61-80`, `src/slam/SlamPipeline.cc:102-117`
   - **Issue:** `TryInitializeGravityAlignment` initializes roll and pitch from a single accelerometer sample, while `SlamPipeline::initScan` inserts the first 10 scans (`kInitScans`) into the map in the **raw sensor frame** without applying the gravity-aligned rotation (and without voxel downsampling). Averaging the accelerometer over the initialization window and transforming the init scans by the initial roll/pitch aligns the map frame with gravity from scan 0.

4. **Registration health gating before map insertion + robust kernel / planarity check** *(Known open issue)*
   - **Location:** `src/slam/SlamPipeline.cc:90-99`, `src/slam/NormalEstimator.cc:51-64`, `include/mslam/slam/registration/PointDistance.hh:147-170`
   - **Issue:**
     - `SlamPipeline::processScan` inserts `filtered_scan_.points` into `map_` even if `Align()` found zero correspondences. Returning a registration summary (matched count, inlier ratio, RMS, degeneracy status) allows gating map insertion.
     - `NormalEstimator::fromNeighbors` accepts any neighborhood with $\ge 3$ points without checking the eigenvalue ratio ($\lambda_0 / \lambda_1$) for planarity.
     - Point-to-plane residuals have no robust loss (Huber / Cauchy / Geman-McClure) to downweight outlier correspondences near corners or dynamic objects.

5. **Bound `SlamServer::map_history_`** *(Known open issue)*
   - **Location:** `include/mslam/slam/SlamServer.hh:104`, `src/slam/SlamServer.cc:105-116`
   - **Issue:** While the registration `VoxelHashMap` is bounded, `SlamServer::map_history_` retains every scan's `std::shared_ptr<const PointCloud>` increment indefinitely, growing RSS without bound over long runs. Introducing a sliding window or periodic voxel-compacted base snapshot (while maintaining `StreamMap`'s `(epoch, offset)` contract) bounds server memory.

---

### Part 2: `repos/moptim`

#### 2.1 Performance Optimizations

1. **Redundant `setZero()`, upper-triangle mirroring, and full-matrix accumulation in `ChunkedCost` + `IOptimizer`** *(New)*
   - **Location:** `include/moptim/ChunkedCost.hh:58-65, 79-81`, `include/moptim/IOptimizer.hh:151-155, 200-209`, `src/EigenSolver.cc:19-20`
   - **Issue:**
     - `EigenSolver::solve` (`src/EigenSolver.cc:20`) and `ISolver::solve` (`include/moptim/ISolver.hh:17-18`) explicitly require **only the lower triangle** of `A`. Yet `ChunkedCost::computeLinearSystem` (`ChunkedCost.hh:80`) copies the transpose into the strict upper triangle on every call.
     - `IOptimizer::assembleLinearSystem` (`IOptimizer.hh:200-209`) zeroes `hessian_` and `gradient_`, and then each cost writes into scratch buffers `cost_hessian_` (`tangent_dim_ x tangent_dim_`) and `cost_gradient_` (`tangent_dim_`)—which `ChunkedCost` zeroes again (`ChunkedCost.hh:60-61`)—followed by a full `hessian_ += cost_hessian_; gradient_ += cost_gradient_;`.
     - When `active_param_dim_ < tangent_dim_` (e.g., `mslam`'s `ImuRegistration` where `tangent_dim_ = 24` and the point-to-plane cost has `active_param_dim_ = 6`), `ChunkedCost` zeros all $24 \times 24 = 576$ doubles and `IOptimizer` adds all $576$ doubles into `hessian_`, even though only the top-left $6 \times 6$ lower triangle ($21$ entries) is non-zero.

2. **Wasted `std::sqrt`, `LossFunction::evaluate` derivative math, and `scales_` broadcast in `ChunkedCost::computeCost` and `robustify`** *(New)*
   - **Location:** `include/moptim/ChunkedCost.hh:48-49, 98-102, 216-263`, `include/moptim/LossFunction.hh:57-68, 88-94, 113-119`
   - **Issue:**
     - **Dead work in `computeCost(x)`:** When `isRobustified()` is true, `computeCost` (`ChunkedCost.hh:48`) calls `robustify(..., jacobian_.block(0, 0, residuals.size(), 0))` (`jacobian.cols() == 0`). Inside `robustify` (`ChunkedCost.hh:238-252`), for every element `e`, it still computes `scale = std::sqrt(weight)`, `scale *= std::sqrt(rho[1])`, and `std::fill_n(scales_.data() + e * dim, dim, scale)`, all of which is discarded when `jacobian.cols() == 0` (`line 255`).
     - **Two `std::sqrt` calls per element:** When both weights and loss are active (`ChunkedCost.hh:241, 247`), two separate square roots are executed per element instead of `std::sqrt(weight * rho[1])`, and when `loss_ == nullptr`, `std::sqrt(weight)` is recomputed every iteration instead of precomputing once in `setWeights()`.
     - **Unused `out[2]` (`rho''(s)`) in `LossFunction::evaluate`:** `HuberLoss::evaluate`, `CauchyLoss::evaluate`, and `GemanMcClureLoss::evaluate` compute `out[2]` on every element (including an extra division in `HuberLoss`), which is never read (`Known` comment in `LossFunction.hh:18-20`, still computed in hot path).

3. **`ProductManifold::plus` performs redundant block retractions during numerical differentiation** *(New)*
   - **Location:** `include/moptim/NumericalCost.hh:83-95, 151-159`, `include/moptim/Manifold.hh:83-87`
   - **Issue:**
     - In `NumericalCost::fillPointTable` (`NumericalCost.hh:151-159`), `step_delta_` is non-zero in **only a single tangent coordinate `param`**. When the manifold is a `ProductManifold` (e.g., `SE(3) x Euclidean(18)` in `mslam`'s `ImuRegistration`, with 24 tangent dimensions and 48 central-difference points), `ProductManifold::plus` (`Manifold.hh:83-87`) loops over **all** blocks and calls `blocks_[i]->plus(...)` even for blocks whose slice of `delta` is identically zero. For the 36 evaluation points that perturb the Euclidean velocity/bias parameters (`param = 6..23`), `SE3Manifold::plus` is called 36 times with `delta = 0` (`se3Exp(0)` + $3 \times 3$ matrix multiplies) just to copy the unchanged 12 pose doubles.
     - When `isSingleChunk()` is true (`num_elements == 1`, e.g., `ImuPreintegrationFactor`), `NumericalCost` still populates the entire `point_table_` (1,440 doubles) upfront in `prepareJacobian` even though each evaluation point is visited only once.

4. **`NormalEquations.cc`: 128-bit fixed SIMD vector size, low `kMaxFusedCols` on x86-64 AVX2, and $O(\text{cols}^2)$ dot-product passes** *(Known heuristic + New)*
   - **Location:** `src/NormalEquations.cc:18-29, 34-56, 96-159`
   - **Issue:**
     - `Simd<double>::Vector` and `Simd<float>::Vector` (`src/NormalEquations.cc:34-43`) are hardcoded to `__attribute__((vector_size(16)))` (128-bit), even when `__AVX2__` (32 bytes) or `__AVX512F__` (64 bytes) is enabled.
     - On x86-64 with AVX2 (16 YMM registers), `kMaxFusedCols` is `3` (`lines 50-56`), so 6-DOF rigid registration (`cols = 6`) bypasses `accumulateFused` completely and falls back to `accumulateByDotProducts`, even though `passWidth(cols, first)` (`lines 69-82`) already splits `Cols = 6` into 3 register-fitting passes (widths 1, 2, 3) without spilling.
     - For `cols > kMaxFusedCols`, `accumulateByDotProducts` (`lines 18-29`) runs `cols * (cols + 1) / 2 + cols` separate `dot()` passes over memory instead of a runtime-tiled panel loop or `rankUpdate`.

5. **Separate `evaluateResiduals` and `evaluateJacobian` passes in `AnalyticalCost` + runtime inner loop** *(New)*
   - **Location:** `include/moptim/ChunkedCost.hh:73-74`, `include/moptim/AnalyticalCost.hh:59-75`
   - **Issue:**
     - `ChunkedCost::computeLinearSystem` calls `evaluateResiduals` and `evaluateJacobian` in two separate passes over the chunk, causing models like `Point3Distance` and `Point3PlaneDistance` to load `input` and compute `R * p + t` twice per element.
     - `AnalyticalCost::fillJacobian` (`AnalyticalCost.hh:68-73`) uses runtime `params` and `dim` trip counts inside the per-element loop; specializing or branching outside the element loop for `dim == 1` (point-to-plane) and `dim == 3` (point-to-point) eliminates inner loop branching.

6. **Out-of-bounds bug and per-iteration device allocations in `NumericalCostSycl.hh`** *(Known WIP + New bugs)*
   - **Location:** `include/moptim/NumericalCostSycl.hh:31-42, 67-146, 203-216, 240-246`, `CMakeLists.txt:36-37`
   - **Issue:**
     - **OOB bug:** `input_sycl_` is allocated and indexed using `observation_dim_` instead of `input_dim_` (`lines 31-37, 131, 141, 241, 243`), reading out of bounds whenever `input_dim_ != observation_dim_`.
     - **Per-iteration `sycl::malloc_device` / `sycl::free`:** `computeCost` runs 2 device allocations/frees and `computeLinearSystem` runs 6 device allocations/frees plus `param_dim_` `std::make_shared<Model>()` heap allocations on every iteration.
     - **Invalid `Eigen::Map<VectorT>`:** Line 139 passes 3 arguments `(ptr, rows, cols)` to a 1D `Eigen::Map<VectorT>` instead of `Eigen::Map<MatrixT>`.
     - **`EIGEN_DONT_VECTORIZE`:** `CMakeLists.txt:36-37` defines `EIGEN_DONT_VECTORIZE` globally across all CPU translation units when `WITH_SYCL=ON`.

#### 2.2 Code Structure & Correctness Improvements

1. **`LevenbergMarquardt::stepImpl` only checks `CONVERGED` / `SMALL_DELTA` on rejected trials, never on accepted steps** *(New)*
   - **Location:** `src/LevenbergMarquardt.cc:48, 93-111`, `src/GaussNewton.cc:58-66`
   - **Issue:**
     - In `LevenbergMarquardt::stepImpl`, `isDeltaSmall` and `isCostSmall` are checked **only inside `if (rho < 0 || std::isnan(rho))`** (when a trial step is rejected). When a trial step is **accepted** (`lines 106-110`), `LevenbergMarquardt` unconditionally returns `{Status::STEP_OK, trial_cost}` even if `isCostSmall(trial_cost)` or `isDeltaSmall(delta_.data(), delta_.size())` is true, forcing an extra full linear system assembly (`JTJ`, `JTb`) at the converged point.
     - When `rho < 0 || std::isnan(rho)` and `isDeltaSmall` is true (`lines 96-98`), `x` is not updated to `trial_x_`, yet line 96 checks `isCostSmall(trial_cost)` instead of `isCostSmall(initial_cost)` and line 97 emits `Phase::FINISHED` with `trial_cost` while line 98 returns `{status, initial_cost}`.
     - Line 48 (`lambda_ = kInitialLambdaFactor * diagonal_.cwiseAbs().maxCoeff();`) can set `*lambda_ = 0` if `diagonal_` is zero, keeping `*lambda_ *= nu` stuck at `0`.

2. **Constructor/member duplication between `GaussNewton` and `LevenbergMarquardt` & missing `ISolver` dimension check** *(New)*
   - **Location:** `include/moptim/GaussNewton.hh:11-31`, `src/GaussNewton.cc:7-22, 51-56`, `include/moptim/LevenbergMarquardt.hh:11-49`, `src/LevenbergMarquardt.cc:11-30`, `include/moptim/ISolver.hh:7-27`
   - **Issue:** Both optimizers duplicate `solver_`, `delta_`, and 4 constructor overloads, and neither validates that `solver != nullptr` or `solver->dimensions() == tangent_dim_` (passing a solver sized to `ambientDim()` causes out-of-bounds memory access in `EigenSolver::solve`).

3. **Unused `Phase` enum values (`Phase::LINEAR_SYSTEM`, `Phase::TRIAL`)** *(New)*
   - **Location:** `include/moptim/Observer.hh:14-20`
   - **Issue:** `Phase::LINEAR_SYSTEM` and `Phase::TRIAL` are never emitted anywhere in `moptim` (`onLinearSystem` uses `LinearSystemEvent<T>`), forcing observer `switch` statements to handle dead cases.

4. **Non-configurable optimizer hyperparameters and convergence tolerances** *(New)*
   - **Location:** `include/moptim/Convergence.hh:10-28`, `include/moptim/LevenbergMarquardt.hh:32-34`, `include/moptim/NumericalCost.hh:161`
   - **Issue:** Step-norm tolerance, cost tolerance, LM initial damping factor (`1e-7`), max trials per iteration (`3`), and finite-difference step size (`sqrt(eps)`) are hardcoded constants with no setters on `IOptimizer` / `LevenbergMarquardt`.

5. **Enforce `const M&` for `residual()` and `jacobian()` in `IModel` concepts** *(New)*
   - **Location:** `include/moptim/IModel.hh:36-50`
   - **Issue:** `ResidualModel` and `AnalyticalModel` check `m.residual(...)` and `m.jacobian(...)` on a mutable `M& m` instead of `const M&`, even though `IModel.hh:20-23` requires them not to mutate shared model state after `setState()`.

6. **Test and CMake cleanup** *(New)*
   - **Location:** `test/test_simple_model.cc:54-88`, `test/transform3d.hh:1, 7, 78`, `test/test_curve_fitting.cc:59-103`, `test/bench_rankupdate.cc`, `test/playground.cc`, `cmake/code-format.cmake:6-10`, `AGENTS.md:3, 18`
   - **Issue:**
     - `TYPED_TEST(SimpleModelTest, LevenbergMarquardt)` and `LevenbergMarquardtAnalytical` (`test/test_simple_model.cc:54-88`) are identical copy-pastes that both test `AnalyticalCost`, leaving `NumericalCostForwardEuler` untested in `SimpleModelTest`.
     - `test_curve_fitting.cc:59-103` runs 10,000 benchmark iterations inside the unit test binary.
     - `cmake/code-format.cmake:6` uses `CMAKE_SOURCE_DIR` instead of `CMAKE_CURRENT_SOURCE_DIR` and globs into `build/`.

#### 2.3 Architectural Improvements

1. **Generalize `active_param_dim` from leading-prefix-only to `(offset, dim)` or block indices** *(New)*
   - **Location:** `include/moptim/ICost.hh:27-46, 80-82, 104-123`, `include/moptim/ChunkedCost.hh:64-65`, `include/moptim/NumericalCost.hh:99-113, 138-159`, `include/moptim/Manifold.hh:65-95`
   - **Issue:** `active_param_dim` can only select a leading prefix `[0, active_param_dim)` of the tangent vector. Any cost depending on a non-leading block of a `ProductManifold` (e.g., velocity/bias priors in `mslam` or `OffsetAnchor` in `test_manifold.cc:348-360`) must declare all preceding tangent dimensions active and pad Jacobians with zeros. Supporting an `(active_param_offset, active_param_dim)` slice (or active block list on `ProductManifold`) allows both analytical and numerical costs to touch only the relevant manifold block(s).

2. **Direct accumulation into optimizer normal equations (`accumulateLinearSystem`)** *(New)*
   - **Location:** `include/moptim/ICost.hh:60-73`, `include/moptim/IOptimizer.hh:151-155, 197-220`
   - **Issue:** Having `ICost` provide `accumulateLinearSystem(const T* x, T* JTJ, T* JTb, T& cost)` that adds directly into the active lower-triangular block of the optimizer's `hessian_` and `gradient_` eliminates `IOptimizer::cost_hessian_`/`cost_gradient_`, per-cost `setZero()`, per-cost upper-triangle mirroring, and full-matrix `hessian_ += cost_hessian_`.

3. **Optional fused `residualAndJacobian` concept on `AnalyticalModel`** *(New)*
   - **Location:** `include/moptim/IModel.hh:46-50`, `include/moptim/AnalyticalCost.hh:52-75`
   - **Issue:** Allowing models to optionally provide `residualAndJacobian(x, input, obs, res, jac)` lets `AnalyticalCost` compute both in a single pass without recomputing `R * p + t`.

4. **Expose Hessian / covariance recovery on `IOptimizer` and `ISolver`** *(Known `mslam/todo.md` item + New)*
   - **Location:** `include/moptim/IOptimizer.hh:68-74, 121-122`, `include/moptim/ISolver.hh:7-27`, `include/moptim/EigenSolver.hh:11-20`
   - **Issue:** Exposing `const MatrixT& hessian() const` / `const VectorT& gradient() const` on `IOptimizer<T>` and covariance inversion via `EigenSolver`'s `LLT` decomposition enables downstream degeneracy detection and pose covariance recovery in `mslam`.

---

### Part 3: `repos/msensor`

#### 3.1 Performance Optimizations

1. **Replace `std::deque<Sample>` in `Subscription<T>` with a fixed-capacity ring buffer** *(New)*
   - **Location:** `include/msensor/bus/topic.hh:100-121, 146`
   - **Issue:** `Subscription<T>` has an immutable `capacity_` and a drop-oldest policy (`queue_.pop_front(); ... queue_.push_back(sample);`), but uses `std::deque<Sample>`. As `pop_front()` and `push_back()` advance across `std::deque` chunk boundaries, it allocates and frees heap blocks while holding `m_` on the sensor driver thread.
   - **Fix:** Replace `std::deque<Sample>` with a preallocated `std::vector<Sample>` circular buffer (`head_`, `size_`).

2. **Eliminate redundant condition-variable notifications in `Topic::publish()` and `Subscription::push()`** *(New)*
   - **Location:** `include/msensor/bus/topic.hh:115-120, 194-210, 222-228`
   - **Issue:**
     - `Subscription<T>::push()` calls `cv_.notify_one()` on every sample even when `hooked_` is true (`SensorStreamReactor` and `BusRecorder` use `setWakeHook()` + `tryPop()` and never block in `Subscription::pop()`).
     - `push()` does `pending_.store(true, std::memory_order_release); return true;`, returning `true` even when `pending_` was **already `true`**. Consequently, `TopicCore<T>::publish()` calls `notifyDispatcher()` (locking `wake_m_` and calling `wake_cv_.notify_one()`) on every sample in a burst even when the dispatcher is already scheduled.
   - **Fix:** Return `!pending_.exchange(true, std::memory_order_acq_rel)` when `hooked_` is true, and only call `cv_.notify_one()` when not hooked (or when a waiter is present).

3. **Avoid redundant zero-initialization and per-scan allocation in point-cloud conversions** *(New)*
   - **Location:** `include/msensor/interface/ILidar.hh:16-21`, `src/conversions/lidar_imu_conversions.cc:8-34`, `src/lidar/mid360.cc:37-43`
   - **Issue:** `fromProtobuf(const sensors::PointCloud3 &)` allocates a new `Scan3DI` on every call and `scan.points.resize(msg.x_size())` zero-initializes all `16 * N` bytes right before overwriting every element. Adding an output-parameter overload `void fromProtobuf(const sensors::PointCloud3 &msg, Scan3DI &scan)` allows callers to reuse buffer capacity.

4. **Coalesce `write()` calls and batch `flush()` in `ScanRecorder` / `BusRecorder`** *(New)*
   - **Location:** `src/recorder/scan_recorder.cc:45-63`, `src/recorder/bus_recorder.cc:165-195`
   - **Issue:**
     - `ScanRecorder::append()` performs two `record_file_->write()` calls (8-byte size prefix, then payload) plus `record_file_->flush()` on every single 36-byte IMU sample (200 Hz).
     - `BusRecorder::drain()` drains batches of IMU and LiDAR samples on a single writer thread, acquiring `ScanRecorder::mutex_` and calling `flush()` on every sample in the loop instead of once at the end of `drain()`.

5. **Eliminate per-frame allocations and per-client redundant JPEG encoding in camera pipeline** *(New)*
   - **Location:** `src/conversions/camera_conversions.cc:76-107`, `grpc/camera_service.cc:34-45`, `src/camera/sim_camera.cc:15-18`, `src/camera/opencv_camera.cc:38-40`
   - **Issue:**
     - `camera_conversions.cc:97` heap-allocates `const std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, quality};` on every frame.
     - `CameraServiceImpl::getCameraFrame()` (`grpc/camera_service.cc:40-43`) runs `toProtobuf` inside each client's `SensorStreamReactor`, JPEG-compressing the same raw `CameraFrame` $N$ times when $N$ clients are connected.
     - `SimCamera::startSampling()` (`src/camera/sim_camera.cc:16`) clones its immutable 921.6 KiB `image_` at 30 Hz even though `Topic<CameraFrame>` holds `shared_ptr<const CameraFrame>`.

6. **Add `MADV_SEQUENTIAL` to `ScanPlayer` and add `Subscription::clear()` for `SensorStreamReactor::OnCancel()`** *(New)*
   - **Location:** `src/recorder/scan_player.cc:35-42`, `grpc/sensor_stream.hh:129-135`
   - **Issue:**
     - `ScanPlayer` `mmap`s `.pbscan` files for sequential reads without calling `madvise(..., MADV_SEQUENTIAL)`.
     - `SensorStreamReactor::OnCancel()` drains `sub_` via `while (sub_->tryPop()) {}`, locking and unlocking `Subscription::m_` up to 4,000 times while holding `SensorStreamReactor::m_`.

7. **Vectorize Python client intensity-to-RGB mapping and throttle 200 Hz IMU UI updates** *(New)*
   - **Location:** `client/client.py:48-49, 134, 143-156, 241`
   - **Issue:** `[int2rgb(int(value)) for value in intensity]` runs ~9,600 Python function calls per scan (~200k calls/s at 21 Hz) instead of indexing a precomputed `256x3` NumPy lookup table, and `stream_imu()` pushes 1,000-element plot arrays to `viser` and logs at `INFO` at 200 Hz.

#### 3.2 Code Structure & Correctness Improvements

1. **Bug: `ICM20948::readSample()` outputs accelerometer data in $g$ instead of $\text{m/s}^2$** *(New)*
   - **Location:** `include/msensor/interface/IImu.hh:15-17`, `include/msensor/imu/icm-20948_defs.h:60-67`, `src/imu/icm-20948.cc:241`
   - **Issue:** `IImu.hh` specifies `ax, ay, az` in $\text{m/s}^2$ and `icm-20948_defs.h:65` defines `FACTOR_ACC_2G_MS = FACTOR_ACC_2G * GRAVITY`, but `ICM20948::readSample()` (`src/imu/icm-20948.cc:241`) calls `convert_raw_data(acc_data, FACTOR_ACC_2G)`, leaving `FACTOR_ACC_2G_MS` unused.

2. **Bug: Signed I2C SMBus error codes truncated to unsigned in `ICM20948` and `ADS1115`** *(New)*
   - **Location:** `src/imu/icm-20948.cc:24-29, 43-46`, `src/adc/ads1115.cc:97-102, 127-130, 142-145`
   - **Issue:**
     - `ICM20948::write_()` and `read_()` return `uint8_t`, turning `i2c_smbus_*` error `-1` into `255` (`0xFF`), so `int res = write_(...); if (res < 0)` in `bank_select()` never triggers.
     - `ADS1115::read_()` casts `i2c_smbus_read_word_data` (`int32_t`, `< 0` on error) directly to `uint16_t`, so `ADS1115::readSingleEnded()` never returns `std::nullopt` on hardware read errors.

3. **Bug: Transient I2C read error in `ICM20948` terminates the process** *(New)*
   - **Location:** `src/imu/icm-20948.cc:72-79, 230-232`, `include/msensor/interface/SamplingThread.hh:80-95`
   - **Issue:** `ICM20948::readSample()` throws `std::runtime_error` on a short I2C block read inside `SamplingThread::runPeriodic()` with no `try/catch`, causing `std::jthread` to call `std::terminate()`.

4. **Bug: Dangling stack `std::promise` on timeout and unchecked null `response` in `Mid360`** *(New)*
   - **Location:** `src/lidar/mid360.cc:94-113, 125-144`
   - **Issue:** `Mid360::setMode()` and `setScanPattern()` pass a stack-allocated `std::promise<void>` to `SetLivoxLidarWorkMode` / `SetLivoxLidarScanPattern`. If `future.wait_for(1000ms)` times out and throws, the stack `promise_complete` is destroyed while the Livox SDK still holds its pointer (unlike `init()`, which uses member `info_promise_`). Both callbacks also dereference `response->ret_code` without checking `if (response != nullptr)`.

5. **Bug: `AttributeError` in `client/client.py` ADC debug log and uninitialized `bg_img`** *(New)*
   - **Location:** `client/client.py:271, 314-315`, `client/min_client.py:100-101`
   - **Issue:** `client.py:271` accesses `resp.timestamp` instead of `resp.header.timestamp`, and `np.ndarray((1, 3, 3))` allocates uninitialized `float64` memory instead of `np.zeros((1, 3, 3), dtype=np.uint8)`.

6. **Lifetime & thread-safety fixes (`RecordingSensorDriver`, `Subscription::setWakeHook`)** *(New)*
   - **Location:** `include/msensor/recorder/recording_driver.hh:68-80`, `src/recorder/recording_driver.cc:24-36`, `include/msensor/bus/topic.hh:309-315`
   - **Issue:**
     - `RecordingSensorDriver` declares `std::jthread thread_;` at line 68 *before* `control_mutex_`, `control_cv_`, `scan_callback_`, and `imu_callback_`; `thread_` should be declared last so it is joined before other members are destroyed.
     - `RecordingSensorDriver::stopSampling()` does not reset `started_` to `false`, preventing restart.
     - `Subscription::setWakeHook({})` sets `hooked_.store(true)` instead of `hooked_.store(has_hook)`.

7. **Deduplicate helpers, remove dead types, and fix CMake/header hygiene** *(New)*
   - **Location:** `include/msensor/imu/icm-20948_defs.h:6-67`, `cmake/format.cmake:5`, `src/config/CMakeLists.txt:5-7`, `include/msensor/interface/ILidar.hh:10-14, 23, 30-34`, `src/recorder/scan_checker.cc`
   - **Issue:**
     - Deduplicate `FeedSensor` (3 tests), `onlySensor()` (3 gRPC services), 1 Hz log throttler (4 places), signal handlers (`playback_publisher.cc`, `remote_recorder.cc`), and I2C open/slave setup (`ads1115.cc`, `icm-20948.cc`).
     - Move global macros/constants in `icm-20948_defs.h` into `namespace msensor` and include `*.h` in `cmake/format.cmake`.
     - Change `src/config/CMakeLists.txt:5-7` from `PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..` to `PRIVATE`.

#### 3.3 Architectural Improvements

1. **Merge-drain IMU and LiDAR in timestamp order in `BusRecorder::drain()`** *(New)*
   - **Location:** `src/recorder/bus_recorder.cc:165-195`
   - **Issue:** In each `BusRecorder::drain()` pass, the writer writes **all** queued IMU samples before **all** queued LiDAR scans (`pass(impl_->imu, imus); pass(impl_->lidar, scans);`). After a disk stall where multiple scans and dozens of IMU samples accumulate, scans are written into the `.pbscan` file *after* IMU samples that occurred up to hundreds of milliseconds later. Adding `Subscription::peek()` and merge-draining by `header.timestamp` preserves chronological ordering in recordings.

2. **Align timestamp clock domains across sensors (`steady_clock` vs PTP/`system_clock`)** *(New)*
   - **Location:** `include/msensor/timing/timing.hh:21-33`, `src/timing/timing.cc:7-30`, `src/lidar/mid360.cc:203, 230`
   - **Issue:** `Mid360` stamps hardware PTP (`system_clock` epoch) timestamps, whereas `OpenCvCamera`, `ICM20948`, `RPLidar`, and `ADS1115` call `timing::getNowNs()` (`steady_clock`), while `timing::getEpochTimeNs()` (`system_clock`, documented for PTP alignment) is unused.

3. **Unify `SensorsRemoteClient` lifecycle with `ILidar`/`IImu` and subscribe lazily** *(New)*
   - **Location:** `grpc/sensors_remote_client.hh:32-50`, `grpc/sensors_remote_client.cc:26-36, 138-172`
   - **Issue:** `SensorsRemoteClient` overrides `startSampling()` / `stopSampling()` as no-ops and opens both LiDAR and IMU gRPC streams unconditionally in `start()`, even when the caller only registered a callback for one sensor.

4. **Batch high-rate IMU samples over gRPC** *(Known open issue in `todo` and `docs/sensor-bus-proposal.md`)*
   - **Location:** `proto/imu.proto`, `grpc/imu_service.cc`, `grpc/sensors_remote_client.cc`
   - **Issue:** Sending 1 protobuf message per IMU sample at 200 Hz incurs high gRPC per-message framing and wake-up overhead. Adding an optional batched streaming RPC (or repeated field) while keeping `getImuData` for backward compatibility will significantly reduce CPU usage on the Pi 5.

5. **Reject unknown JSON keys in `msensor::Config::fromFile()` and expose missing server/driver settings** *(New)*
   - **Location:** `src/config/config.cc:125-318`, `src/apps/sensor_publisher.cc:65-67`, `src/factory/sensor_factory.cc:104`
   - **Issue:** `Config::fromFile()` silently ignores unknown keys (masking typos in `publisher_config.json`), and hardcodes the server listen address (`0.0.0.0:50051`), `max_clients_per_stream`, and RPLidar RPM (`360`).

