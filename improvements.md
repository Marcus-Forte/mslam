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
