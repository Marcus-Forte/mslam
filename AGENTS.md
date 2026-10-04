# mslam

C++20 LiDAR/IMU SLAM (Eigen, jsoncpp, gRPC). Entrypoint `src/main.cc`; libs in `src/{slam,map,config}`; proto in `grpc/slam.proto`; architecture notes in `docs/arch.md`.

## Build & test
- Configure/build via presets: `cmake --preset gcc && cmake --build --preset build-gcc` (Ninja, toolchain `/opt/toolchain/gcc.cmake`, output `build/gcc/`). The readme's `build/default/` paths are stale; use `build/gcc/`.
- The default build runs a `format-mslam` target (`clang-format -i` on all non-third_party `.cc/.hh`), so builds rewrite source files. Expect formatting diffs.
- Tests are GTest in `test/` (`test_slam`, `test_preprocessor`, `test/map`): run `ctest --test-dir build/gcc`, or a single binary with `--gtest_filter=...`.
- Integration tests replay recordings through the full pipeline from `test/integration/` (skipped if the untracked recording is absent). They now start a gRPC replay server and consume it through the remote client. They are opt-in: default `cmake`/`ctest` never build or run them. Enable with `cmake --preset gcc -DMSLAM_INTEGRATION_TESTS=ON`, then `ctest --test-dir build/gcc -L integration`. Or run directly with `cmake --build --preset build-gcc --target integration-test`.
- `register_scans2d` is `EXCLUDE_FROM_ALL`; build it explicitly with `--target register_scans2d`.
- The replay server lives in `third_party/msensor` (`playback_publisher`); build it with `--target replay-server` (or `playback_publisher`). It is placed next to `mslam` in `build/gcc/`. Install it with `-DMSLAM_INSTALL_REPLAY_SERVER=ON` (off by default, so deployment images omit it).
- CPU target is set globally via `MSLAM_CPU` (default `native`; the `pi5` preset and `deployment/Dockerfile` use `cortex-a76`). It must be global for Eigen ABI consistency across TUs; don't remove or apply per-target. The `gcc` preset's toolchain adds `-march=armv8-a`, which overrides `-mcpu`'s ISA, so use `pi5` for Pi deployments.

## Running
- The sensor source is chosen by `remote_scanner` in the config: `"local"` uses the on-board Mid360, any other value is a `host:port` of a sensor server.
- Replay a recording as a separate sensor server, then point `mslam` at it:
  `build/gcc/playback_publisher -f <file>.pbscan [-s <speed>] [-p <port>] [--autoplay]` (`speed` 1.0 = real time, 0 = max; default port 50051), and in another terminal `build/gcc/mslam -c config/mslam.jsonc` with `remote_scanner` set to the server. Sample recordings are in the repo root (`living_room.pbscan`, `eratolaan.pbscan`).
- Root `*.ply` / `*_out.pbscan` files are generated outputs.
- The `regression` skill covers the SLAM regression check.

## Repo layout gotchas
- `third_party/` are submodules (`moptim` optimizer, `msensor` sensors/recorder, `robin-map`); `msensor` is maintained in-house (its recording/playback and gRPC code is fair game), `moptim`/`robin-map` are external. Submodules show as modified in git status.
- `client/` is a Python viewer (uv, `proto_gen/` generated from `grpc/slam.proto`); `viewer/` is a TypeScript browser PLY viewer (`npm run dev`).
- Docker builds: `deployment/Dockerfile` (SLAM), `deployment/DockerfileViewer`.
