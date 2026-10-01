# mslam

C++20 LiDAR/IMU SLAM (Eigen, PCL, jsoncpp, gRPC). Entrypoint `src/main.cc`; libs in `src/{slam,map,config}`; proto in `grpc/slam.proto`; architecture notes in `docs/arch.md`.

## Build & test
- Configure/build via presets: `cmake --preset gcc && cmake --build --preset build-gcc` (Ninja, toolchain `/opt/toolchain/gcc.cmake`, output `build/gcc/`). The readme's `build/default/` paths are stale; use `build/gcc/`.
- The default build runs a `format-mslam` target (`clang-format -i` on all non-third_party `.cc/.hh`), so builds rewrite source files. Expect formatting diffs.
- Tests are GTest in `test/` (`test_slam`, `test_preprocessor`, `test/map`): run `ctest --test-dir build/gcc`, or a single binary with `--gtest_filter=...`.
- `register_scans2d` is `EXCLUDE_FROM_ALL`; build it explicitly with `--target register_scans2d`.
- Compiled with `-march=native` globally (needed for Eigen ABI consistency across TUs); don't remove or apply per-target.

## Running
- Playback: `build/gcc/mslam -c config/mslam.json -f <file>.pbscan -d <ms> [-o out/prefix]`. Sample recordings are in the repo root (`living_room.pbscan`, `eratolaan.pbscan`).
- Root `*.ply` / `*_out.pbscan` files are generated outputs.
- The `regression` skill covers the SLAM regression check.

## Repo layout gotchas
- `third_party/` are submodules (`moptim` optimizer, `msensor` sensors/recorder, `robin-map`); `msensor` has its own `AGENTS.md`. Don't edit them casually; they show as modified in git status.
- `client/` is a Python viewer (uv, `proto_gen/` generated from `grpc/slam.proto`); `viewer/` is a TypeScript browser PLY viewer (`npm run dev`).
- Docker builds: `docker/Dockerfile` (SLAM), `docker/DockerfileViewer`.
