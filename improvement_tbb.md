> TBB is mslam's parallel path again (2026-11-24); it was briefly replaced by SYCL (2026-11-22), which has been removed. The study below describes the code as restored.

# oneTBB in mslam: parallel correspondence search and normal estimation

Status (2026-11-16): implemented for the two per-point loops that read a map
nobody writes during `Align`: the 1-NN voxel search (`CorrespondenceFinder::find`)
and the surface normal estimation of map points not cached yet
(`NormalEstimator::estimateAll`, driven by `IcpWorkspace::findPlaneConstraints`).
The default stays sequential (`num_threads: 1`). Results are bit-identical at any
thread count. Everything below marked "measured" was measured on the machine
described in section 4; anything else is marked "not measured".

This file started as a study. Section 5 lists what in that study did not hold.

---

## 1. How to use it

- `slam.num_threads` in `config/mslam.jsonc` (optional, default 1),
  `registration.num_threads` in `config/registration.jsonc` (default 1), and
  `bench_slam --threads <n>`. 1 = the sequential code path (no executor at all),
  0 = every hardware thread, n = at most n threads, the calling thread included.
  Accepted range 0 to 256; a non-integer or negative value is rejected.
- Build: `MSLAM_WITH_TBB` (CMake option, default ON) looks for oneTBB with
  `find_package(TBB QUIET)`. Not found: a CMake warning and a sequential build.
  In a sequential build a `num_threads` other than 1 logs one warning per
  `Slam` / `ScanSequenceRegistration` and runs sequentially.
- Packages: `libtbb-dev` to build (in `repos/cpp-dev/Dockerfile` since commit
  330c6c1; the local `mdnf1992/cpp-dev` image re-created on 2026-11-16 has
  oneTBB 2023.1.0), `libtbb12` at run time (added to the runtime stage of
  `deployment/Dockerfile`, because the install rule skips `/usr/lib`).
  A container started from an older image needs
  `apt-get update && apt-get install -y libtbb-dev`.

---

## 2. Design as built

### 2.1 One bounded executor (`include/mslam/utils/Parallel.hh`, `src/utils/Parallel.cc`)

- `ParallelExecutor(unsigned num_threads)` owns a `tbb::task_arena` of that many
  slots (0: `tbb::info::default_concurrency()`), created in the constructor
  (`arena.initialize()`), so the first scan does not pay for it. The caller
  occupies one slot. TBB headers stay in the `.cc` (pimpl); `slam` links
  `TBB::tbb` and defines `MSLAM_HAS_TBB` privately, so no public header depends
  on TBB and the public headers stay C++20.
- `forRange(n, grain, body)` calls `body(begin, end)` over disjoint sub-ranges of
  `[0, n)` through `arena.execute` + `tbb::parallel_for(blocked_range(0, n, grain),
  ..., tbb::static_partitioner{})`. It runs `body(0, n)` inline on the caller when
  the executor has one thread or `n < 2 * grain`. The body is passed by a
  function pointer plus a context pointer: no allocation per region.
- `makeParallelExecutor(n, logger)` returns `nullptr` for 1 (the callers then run
  their original loops unchanged) and, with one warning, in a build without TBB.
- No `tbb::global_control` in library code.

The rule that makes results independent of the thread count: a parallel body
writes only the output slot of its own index. Compaction, cache insertion and
constraint assembly stay sequential and in the original order. Nothing is summed
in parallel.

### 2.2 1-NN search (`CorrespondenceFinder`)

- `CorrespondenceFinder(std::shared_ptr<const ParallelExecutor> executor = nullptr)`.
  Without an executor the original loop runs.
- With one: `nearest_[i] = map.getClosestNeighbor(scan[i])` in parallel into a
  scratch vector (a `mutable` member, so one `find()` at a time per finder;
  every registration owns its finder), then one sequential pass appends
  `{scan[i], nearest_[i].first}` where `nearest_[i].second < gate^2`: the same
  predicate, the same order.
- `createLoggingCorrespondenceFinder(logger, executor = nullptr)` forwards it.

### 2.3 Normal estimation (`NormalEstimator`, `IcpWorkspace`)

- The cache is `robin_map<MapPointKey, uint32_t slot>` plus
  `std::vector<std::optional<Eigen::Vector3d>> normals_`. Keeping the normals
  out of the hash map lets a parallel phase fill slots without touching it.
- `estimateAll(correspondences, slots, executor)`:
  1. sequential: one `try_emplace` per correspondence resolves its slot; a new
     key gets a new slot (`normals_` grows here and only here) and goes into a
     `pending_` list;
  2. parallel over `pending_`: `normals_[slot] = fromNeighbors(findClosestNeighbors(...))`
     with a stack neighbour array. In rounds 2 and 3 of an `Align` the list is
     short (most map points are cached), and lists below twice the grain run
     inline.
- `IcpWorkspace::findPlaneConstraints` then assembles `plane_inputs_` /
  `plane_offsets_` sequentially in correspondence order from the slots, with the
  same expressions as before.
- `estimate(point)`, `cachedCount()` and `reset()` keep working on the same
  storage.
- `IcpWorkspace`, `PointToPlaneRegistration` and `ImuRegistration` take a
  trailing `executor = nullptr`. Existing call sites compile unchanged.

### 2.4 Contract (`include/mslam/map/IMap.hh`)

`getClosestNeighbor` and `findClosestNeighbors` must be safe to call from several
threads while no non-const method runs; `getPointCloudRepresentation()` is
excluded (it rebuilds a `mutable` cache). `VoxelHashMap` meets this: both search
methods only read `map_`, `pool_` and `voxel_shifts_`.

### 2.5 Wiring

- `Slam` creates one executor (declared before `registration_`, members are in
  initialization order) and hands it to both registrations and both finders.
- `ScanSequenceRegistration` does the same for `registration_` and `fit_`;
  `register_scans` copies `registration.num_threads` into
  `ScanSequenceParameters::num_threads`.
- `SlamParameters::num_threads` is printed in the config dump (`Threads: n`).

### 2.6 Grain and partitioner (measured, section 4.4)

- Search: grain 128 scan points (64, 128 and 256 measured the same).
- Normals: grain 16 new map points (better than 32 and 64; 8 no better).
- `static_partitioner`: one contiguous block per thread, no stealing. Faster than
  `auto_partitioner` and `simple_partitioner` at 4 threads. At 6 threads
  `auto_partitioner` was faster (section 4.4); revisit if a default above 4 is
  chosen.

---

## 3. Verification

All in the `mslam-claude` container (image `mdnf1992/cpp-dev`, GCC 16, aarch64,
`gcc` preset: Release), oneTBB 2023.1.0 from Debian (`libtbb-dev` 2023.1.0-3).

### 3.1 Bit identity (measured)

`bench_slam test/integration/mslam.jsonc test/data/<rec>.pbscan 1 --threads N`
against the binary built from the tree before the change (kept in
`/tmp/tbb-base/` in the container):

| Recording | Binary | Final pose | |
|---|---|---|---|
| living_room_garden | baseline | `0.043671476 -0.000098382 0.001114079 -0.003311060 0.002207124 0.487497048` | reference |
| living_room_garden | N = 1, 2, 4, 8 | same | identical |
| rooms | baseline | `-0.015176207 -0.116095051 -0.004324399 0.000635034 0.001027378 -0.031378821` | reference |
| rooms | N = 1, 2, 4, 8 | same | identical |
| living_room_garden, preprocessor voxel 0.05 | baseline and N = 4 | `0.043058459 0.000714589 0.000448311 -0.003277757 0.001715949 0.487989961` | identical |

`register_scans -c <config with num_threads N> --poses test/data/registration/poses.jsonc test/data/registration/{a,b,c}.ply`
prints, at N = 1, 2, 4 and 8, the reference poses of `AGENTS.md` (b
`0.099908569 -0.060199187 0.040658667 0.009875589 -0.014945448 0.029959427`, c
`1.499873182 -0.800181006 0.200337936 -0.000066134 0.000016581 0.499981177`),
and the same matched counts, RMS and first-round corrections as the baseline.

The printed poses have 9 digits. Bitwise equality is asserted by tests:
`SlamIntegration.FourThreadsMatchTheSynchronousReplay` (the whole
`living_room_garden` replay at 4 threads, `EXPECT_EQ` on the doubles),
`PointToPlaneRegistration.ParallelAlignMatchesSequentialBitForBit`,
`ParallelCorrespondenceFinder.*` and `NormalEstimator.EstimateAllMatchesPerPointEstimates`.

### 3.2 Tests

- New `test/test_parallel.cc`: every index visited exactly once (sizes around
  the grain, 1/2/4/all threads), one thread and small ranges run on the caller,
  concurrency bounded by the request, `makeParallelExecutor`.
- `test_correspondence_finder.cc`: 4-thread finder equals the sequential one
  element for element (bitwise) on a `VoxelHashMap`; reuse with a smaller scan;
  empty scan; all rejected; below the inline threshold.
- `test_normal_estimator.cc`: `estimateAll` on 4 threads equals per-point
  `estimate` bitwise, with duplicate map points, points without a normal and
  cache reuse across two calls; the sequential path and `reset`.
- `test_registration.cc`: two consecutive `Align` calls on 4 threads give
  bitwise the sequential poses.
- `test_json_config.cc`, `test_registration_config.cc`: `num_threads` default,
  parsing (including 0), invalid values rejected.
- `ctest --test-dir build/gcc`: 224 of 224 passed (integration test included).

### 3.3 Build without TBB (measured)

A scratch build with `-DMSLAM_WITH_TBB=OFF`: `test_parallel`,
`test_correspondence_finder`, `test_normal_estimator`, `test_registration`,
`test_json_config` pass (the 4-thread comparisons skip; the "built without TBB"
warning is asserted by `ParallelExecutor.MakeReturnsNullForOneThread`),
`bench_slam` does not link TBB, and `bench_slam ... --threads 4` prints the
reference pose of `living_room_garden`. The integration test skips
`FourThreadsMatchTheSynchronousReplay` and passes the other three.

### 3.4 Thread sanitizer (measured, limited)

Scratch build with `-fsanitize=thread` (`-O1 -g`), Debian's oneTBB (not
instrumented). `test_parallel`, `test_correspondence_finder`,
`test_normal_estimator` and `test_registration` all pass, and TSan reports 64,
91, 16 and 44 races. Grouped by the first mslam frame of each access, every one
is either inside TBB alone (no mslam frame in either access) or pairs an access
in a parallel body on a worker with a main-thread access ordered by the region's
fork or join: building the map (`VoxelHashMap::addScan`) or the scan before the
region, `nearest_.resize` and the phase-1 slot allocation in `estimateAll`
before it, the compaction after it, the next round's `resetSource` /
`moveSource`, and the scratch slots of one region written again in the next
region (the same line on both threads). TSan cannot see TBB's fork/join
synchronization because TBB is not instrumented, so it reports all of these.
No report showed two bodies of one region touching the same location; that
disjointness is what `ParallelForRange.VisitsEveryIndexExactlyOnce` checks
directly. A clean TSan run needs an instrumented oneTBB build (not done).

---

## 4. Measurements

Machine: Apple M3 Pro (5 performance + 6 efficiency cores), Docker VM with 11
vCPUs, `mslam-claude` container, GCC 16 Release (`gcc` preset). The host is shared
and noisy: load averages are given per measurement (from `uptime` in the VM).
Unless stated otherwise: `bench_slam test/integration/mslam.jsonc <rec> 1`,
5 rounds, each round runs every variant once (baseline and new interleaved),
minimum over the rounds. Times in ms.

### 4.1 `bench_slam` (measured)

`living_room_garden` (1129 scans), load 3.6 to 2.9:

| Binary | total | register | total speed-up | register speed-up |
|---|---|---|---|---|
| baseline | 1945.5 | 1569.2 | 1.00 | 1.00 |
| N = 1 | 1889.5 | 1527.1 | 1.03 | 1.03 |
| N = 2 | 1340.1 | 962.5 | 1.45 | 1.63 |
| N = 4 | 1049.8 | 669.9 | 1.85 | 2.34 |
| N = 8 | 1179.0 | 745.4 | 1.65 | 2.11 |

`rooms` (831 scans), load 2.9 to 3.1:

| Binary | total | register |
|---|---|---|
| baseline | 906.6 | 682.0 |
| N = 1 | 899.2 | 678.7 |
| N = 2 | 663.0 | 442.0 |
| N = 4 | 544.6 | 318.7 |
| N = 8 | 678.3 | 395.8 |

N = 1 is not slower than the baseline (its minima are 0.5 to 3% lower; I do not
read a gain into that on this host). The rest of the total (preprocess about
0.3 s, map insert, predict) is untouched, which bounds the total speed-up.

The study expected a ceiling of 1.5x / 2.1x / 2.6x on the total at 2 / 4 / 8
threads (70% parallel work). Measured: 1.45x / 1.85x / 1.65x. The register stage
alone gained more than the earlier stage split suggested (2.34x at 4 threads),
so the parallel share of register is larger than the old profile's split
implied; not re-profiled.

Beyond 4 threads it gets slower here. Measured with the static and the auto
partitioner alike (section 4.4), so it is not the partitioner. A likely
explanation, not verified: the M3 Pro has 5 performance cores, the host keeps
some of them busy, and above 4 threads part of each region lands on efficiency
cores, which every region then waits for.

### 4.2 Denser preprocessing (measured)

`test/integration/mslam.jsonc` with `preprocessor.voxel_size` 0.05 instead of
0.1 (scratch copy), `living_room_garden`, load 1.9 to 5.8 (rose during the run):

| Binary | total | register |
|---|---|---|
| baseline | 3071.1 | 2624.0 |
| N = 1 | 3053.1 | 2618.1 |
| N = 2 | 2029.0 | 1621.7 |
| N = 4 | 1530.5 | 1109.1 |
| N = 8 | 1821.5 | 1273.8 |

Register speed-up at 4 threads: 2.37x (2.34x with the default config).

### 4.3 `register_scans` (measured)

`register_scans -c <config> --poses test/data/registration/poses.jsonc a.ply b.ply c.ply`,
wall time of the process (reading the PLYs included), 5 interleaved rounds,
minimum, load 2.9 to 2.5. The baseline ran with the config from `HEAD` (it
rejects the new `num_threads` key).

| Binary | wall (s) | speed-up |
|---|---|---|
| baseline | 2.324 | 1.00 |
| N = 1 | 2.315 | 1.00 |
| N = 2 | 1.388 | 1.67 |
| N = 4 | 0.853 | 2.72 |
| N = 8 | 0.642 | 3.62 |

Whole clouds (45k to 65k thinned points, search radius 3 voxels = 343 voxels per
query) make much larger regions than live scans, so it keeps scaling to 8
threads. The 86 s run on `living_room_garden.pbscan` as one cloud was not
repeated (not measured).

### 4.4 Partitioner and grain (measured)

All at N = 4, `living_room_garden`, from a scratch copy of the tree in which the
partitioner and the two grains were read from environment variables (not part of
the change). Register minima; 5 rounds unless stated.

Partitioner, grains 128 / 32, load 5.9 to 5.3:

| Partitioner | total | register |
|---|---|---|
| sequential (N = 1) | 1889.4 | 1529.0 |
| static | 1029.7 | 655.3 |
| auto | 1050.6 | 677.4 |
| simple | 1068.0 | 685.5 |

Grains, static partitioner, load 4.9 to 4.5 (search / normal grain):

| Search / normal | total | register |
|---|---|---|
| 64 / 32 | 1054.8 | 656.8 |
| 128 / 32 | 1044.7 | 667.7 |
| 256 / 32 | 1040.7 | 665.1 |
| 128 / 16 | 1039.1 | 658.2 |
| 128 / 64 | 1101.8 | 692.1 |
| 512 / 128 | 1201.2 | 826.1 |

Tie-break of the normal grain, 7 rounds, load 4.1 to 4.2:

| Search / normal | total | register |
|---|---|---|
| 128 / 8 | 1023.7 | 647.8 |
| 128 / 16 | 1022.1 | 644.0 |
| 128 / 32 | 1047.8 | 666.0 |
| 64 / 16 | 1017.1 | 645.2 |

Chosen: search 128 (64 to 256 are the same within noise), normals 16 (about 20 ms
of register better than 32 in both runs; the short normal lists of rounds 2 and
3 then still run in parallel), static partitioner.

Above 4 threads, normal grain 16, load 2.8 to 4.3:

| Variant | total | register |
|---|---|---|
| static, N = 4 | 1017.3 | 642.9 |
| auto, N = 4 | 1063.2 | 681.1 |
| static, N = 6 | 1208.5 | 790.3 |
| auto, N = 6 | 1144.7 | 686.1 |
| static, N = 8 | 1144.4 | 725.0 |
| auto, N = 8 | 1203.0 | 739.9 |

### 4.5 Live pacing (measured, few runs)

`bench_slam` runs scans back to back, so the TBB workers stay awake. Live, they
sleep about 46 ms between scans. Measured: `playback_publisher -f
test/data/living_room_garden.pbscan -s 1.0` and `mslam -c <integration config with
remote_scanner localhost:50051 and slam.num_threads N>`, mean of the logged
`Slam::Update elapsed` over the run. Runs alternated N = 1, 4, 1, 4:

| Run | load during run | updates | mean | median | p95 |
|---|---|---|---|---|---|
| N = 1 | 4.4 to 11.5 | 1114 | 6.00 | 5.00 | 13.22 |
| N = 4 | 11.5 to 6.7 | 1108 | 3.56 | 2.67 | 7.97 |
| N = 1 | 6.7 to 3.8 | 1119 | 3.67 | 3.30 | 7.19 |
| N = 4 | 3.8 to 1.5 | 1119 | 2.28 | 2.21 | 3.95 |

`Slam::Update` is the registration of one scan (map insertion is outside it).
In both pairs N = 4 lowered the mean `Update` time by 38 to 41%, against 56% for
the register stage in `bench_slam` (1527.1 to 669.9 ms, section 4.1). So waking
the workers per scan costs part of the gain but leaves most of it. Two runs per
setting on a loaded host: read the direction, not the digits. Under the heavy
load of the first pair the live loop processed fewer scans (1108 / 1114 of
1119). Live `Update` is slower per scan than `bench_slam`'s register stage at
both settings (3.67 against 1.36 ms at N = 1, 2.28 against 0.60 ms at N = 4, in
the quieter pair). The live run logs at info level and its cores idle between
scans; the split was not investigated.

### 4.6 Not measured

- Raspberry Pi 5 (4 Cortex-A76 cores, no efficiency cores): nothing measured. The
  default stays 1 until it is; 4 is the value to try first there.
- `living_room_garden.pbscan` as a single cloud in `register_scans` (86 s before).
- The cost of the sequential compaction and slot passes on their own.

---

## 5. Corrections to the earlier study

- "`libtbb-dev` is not installed, add it to `repos/cpp-dev/Dockerfile`": it is
  in that Dockerfile since commit 330c6c1; only containers started from an older
  image lack it.
- CMake: the study used `find_package(TBB REQUIRED)` and linked `TBB::tbb`
  PUBLIC with a PUBLIC define. As built: `QUIET` with a warning and a sequential
  fallback, PRIVATE link and define, TBB headers only in `Parallel.cc`.
- Switch: a bool (`parallel_search`, `--parallel-search`) became a thread count
  (`num_threads`, `--threads`), so the arena is bounded and 1 means exactly
  today's code. Global `tbb::parallel_for` without an arena would have used every
  core of the machine.
- Grain and threshold: the study proposed `kMinParallelPoints = 256` and grain
  256 for the search. Measured (4.4): the search grain hardly matters between 64
  and 256, and a large normal grain costs time; the inline threshold is
  `2 * grain`.
- The study's "~13 cm drift variance on `rooms`" from a changed correspondence
  order was not verified; the gates in `AGENTS.md` only state that the pose must
  match to the last digit, which this design keeps.
- The study's "compaction < 2 us" was not measured (still not).
- Deskew ("Medium-High, ~2-3x on preprocess"): the per-point orientation in
  `deskewInto` is a running product, so a bit-exact split needs the product
  chain up to each block start first, which is most of the work. All of
  preprocess is about 0.28 to 0.31 ms per scan (measured in 4.1: 0.31 to 0.36 s
  over 1129 scans). Not worth it; filter before rotating instead.
- `VoxelHashMap::prune` with `parallel_sort`: skipped. Eviction is
  order-dependent and only runs with a voxel budget, which the shipped configs
  disable (`max_voxels: 0`).
- LM assembly in moptim `ChunkedCost` with `tbb::parallel_deterministic_reduce`:
  "deterministic" there means repeatable run to run, not equal to today's
  sequential sum. The summation order changes, so the reference poses in
  `AGENTS.md` would move in the last digits and need re-baselining, and the
  change belongs to the moptim repo. In live SLAM a round has about 4 chunks of
  292 elements. Decide only for `register_scans`, after measuring what remains.
- Approach B (fused 1-NN + normal with `enumerable_thread_specific` caches) was
  not taken: per-thread caches recompute normals other threads already have, and
  the slot design (approach A) keeps one cache across the rounds of an `Align`.

---

## 6. Other opportunities (not implemented)

| Candidate | Verdict | Why |
|---|---|---|
| LM assembly in moptim `ChunkedCost` | Later, for `register_scans` only | See section 5; changes reference poses; lives in moptim. |
| Deskew (`Deskew.cc`) | Skip | Running product; preprocess is about 0.3 ms per scan. |
| `VoxelFilter`, `addScan`, `prune` | Skip | Order-dependent hash insertions; a concurrent hash map would slow the reads that dominate. |
| IMU factor numerical Jacobian | Skip | 49 sub-microsecond evaluations; an analytical Jacobian is the fix. |
| Overlapping stages (`parallel_pipeline`) | Skip | Only raises offline replay throughput; the live loop idles between scans. |
| `transformCloud` | Skip for SLAM | About 1 ns per point; for `register_scans` not measured. |

Larger than the parallel normals and composable with them: keeping normals in
the map across scans (open issue in `AGENTS.md`).

Not changed, noticed: `deployment/Dockerfile` installs `libfmt10` in the runtime
stage while the build image has `libfmt11`. Checked in `debian:sid-slim` on
2026-11-16: it is harmless, because `libspdlog1.15` depends on `libfmt11` and
pulls it in; `libfmt10` is only a redundant package. `libtbb12` 2023.1.0-3 is
installable there, the same version as in the build image. The deployment image
itself was not built after this change.

## Sweep 2026-11-25 (TBB restored)

Sweep 2026-11-25 (test runner, Debian sid, GCC 16, x86-64 WSL2 host with 28 hardware threads, `bench_slam` with `test/integration/mslam.jsonc`, minima of 5 interleaved rounds; every pose identical to the old host path). Total ms, `living_room_garden` / `rooms`: old host 3862 / 1814; TBB t1 3890 / 1814, t2 2728 / 1334, t4 2144 / 1086, t8 1998 / 1032, t16 1993 / 1095, t0 (28) 2416 / 1221; the removed SYCL `native_cpu` path 2933 / 1643. `living_room_garden` register split, host -> t8: search 875 -> 215, normals 1522 -> 346, solve 666 -> 669 (moptim inline, one partition). From 16 threads on preprocess and solve get slower (solve 729 at t16, 817 at t0), probably spinning TBB workers hurting the sequential parts, so prefer an explicit count (`config/mslam.jsonc` uses 8; the Pi 5 config keeps 4, not measured) over 0.
