"""`msim` command line."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml


def _floats(s: str) -> tuple[float, float, float]:
    v = tuple(float(x) for x in s.split(","))
    if len(v) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return v  # type: ignore[return-value]


def _add_serve(sub) -> None:
    p = sub.add_parser("serve", help="run the sensor server (viewer or headless)",
                       description="Serve the Mid360 + IMU streams on the msensor gRPC port. With a viewer, "
                                   "fly the sensor with the keyboard (macOS: run under `mjpython -m msim serve`).")
    p.add_argument("--scene", required=True, help="scene YAML")
    p.add_argument("--port", type=int, default=50051)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--headless", action="store_true", help="no viewer; needs --trajectory")
    p.add_argument("--trajectory", help="scripted trajectory YAML (otherwise keyboard control)")
    p.add_argument("--speed", type=float, default=1.0, help="simulated seconds per wall second; 0 = unpaced")
    p.add_argument("--unpaced", action="store_true", help="same as --speed 0 (deterministic, lossless)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--gt-log", help="write the ground-truth pose at each scan's last point (CSV)")
    p.add_argument("--record", help="also write a msensor .pbscan recording")
    p.add_argument("--caster", choices=["box", "mujoco"], default="box")
    p.add_argument("--pattern", default="measured", metavar="{measured,rosette,FILE}",
                   help="LiDAR scan pattern. measured (default): the real Mid360 pattern, measured from "
                        "test/data/living_room_garden.pbscan (cached in out/mid360_pattern.npz, built on first use). "
                        "rosette: synthetic approximation (full field of view every scan). "
                        "FILE: a pattern written by `msim gen-pattern` (.npz) or a .npy table of directions")
    p.add_argument("--range-noise", type=float, default=0.0, help="LiDAR range noise sigma, m")
    p.add_argument("--intensity-noise", type=float, default=0.0,
                   help="relative std of the reported reflectivity (speckle), e.g. 0.1")
    p.add_argument("--imu-accel-noise", type=float, default=0.0, help="accel noise density, m/s^2/sqrt(Hz)")
    p.add_argument("--imu-gyro-noise", type=float, default=0.0, help="gyro noise density, rad/s/sqrt(Hz)")
    p.add_argument("--imu-accel-bias-walk", type=float, default=0.0, help="m/s^2/sqrt(s)")
    p.add_argument("--imu-gyro-bias-walk", type=float, default=0.0, help="rad/s/sqrt(s)")
    p.add_argument("--imu-accel-bias", type=_floats, default=(0.0, 0.0, 0.0), help="constant bias x,y,z in m/s^2")
    p.add_argument("--imu-gyro-bias", type=_floats, default=(0.0, 0.0, 0.0), help="constant bias x,y,z in rad/s")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--wait-for-clients", dest="wait_for_clients", action="store_true", default=None,
                   help="hold the clock until both streams have a client (default with --headless)")
    g.add_argument("--no-wait-for-clients", dest="wait_for_clients", action="store_false")
    p.add_argument("--exit-when-done", action="store_true", help="end the streams and exit when the trajectory ends")
    p.add_argument("--backpressure", choices=["auto", "drop", "block"], default="auto",
                   help="slow client: drop the oldest (real server) or make the simulator wait; auto = block when unpaced")
    p.add_argument("--start-ns", type=int, default=1_000_000_000_000)
    p.add_argument("--show-ui", action="store_true",
                   help="viewer: show MuJoCo's left/right panels (hidden by default; Tab / Shift+Tab toggle them)")
    p.add_argument("--no-points", action="store_true", help="viewer: do not draw the last scan")
    p.set_defaults(_parser=p)


def format_arguments(a, parser) -> str:
    """Every option of a parsed command with its value, marking the ones left at their default."""
    rows = []
    for k, v in vars(a).items():
        if k.startswith("_") or k in ("cmd", "verbose"):
            continue
        default = parser.get_default(k)
        rows.append((k.replace("_", "-"), "-" if v is None else str(v), " (default)" if v == default else ""))
    width = max(len(r[0]) for r in rows)
    return "\n".join(f"  {k:<{width}}  {v}{d}" for k, v, d in rows)


def _cmd_serve(a) -> int:
    from .app import SimOptions, Simulation
    print("msim serve " + " ".join(sys.argv[2:] if len(sys.argv) > 1 and sys.argv[1] == "serve" else []))
    print(format_arguments(a, a._parser), flush=True)
    opts = SimOptions(
        scene=a.scene, port=a.port, host=a.host, trajectory=a.trajectory, headless=a.headless,
        speed=0.0 if a.unpaced else a.speed, seed=a.seed, gt_log=a.gt_log, record=a.record, caster=a.caster,
        pattern=a.pattern, range_noise=a.range_noise, intensity_noise=a.intensity_noise, imu_accel_noise=a.imu_accel_noise,
        imu_gyro_noise=a.imu_gyro_noise, imu_accel_bias_walk=a.imu_accel_bias_walk,
        imu_gyro_bias_walk=a.imu_gyro_bias_walk, imu_accel_bias=a.imu_accel_bias, imu_gyro_bias=a.imu_gyro_bias,
        wait_for_clients=a.wait_for_clients, exit_when_done=a.exit_when_done, backpressure=a.backpressure,
        start_ns=a.start_ns, show_points=not a.no_points, show_ui=a.show_ui)
    sim = Simulation(opts)
    if a.headless:
        sim.run_headless()
    else:
        sim.run_viewer()
    return 0


def _cmd_gen_scene(a) -> int:
    from .scene import load_scene, write_mjcf
    scene = load_scene(a.scene)
    out = a.output or str(Path(a.scene).with_suffix(".xml"))
    write_mjcf(scene, out)
    print(f"wrote {out} ({len(scene.boxes)} boxes)")
    if a.check:
        import mujoco
        mujoco.MjModel.from_xml_path(out)
        print("MuJoCo loads it")
    return 0


def _cmd_gen_traj(a) -> int:
    from .motion import rect_loop
    d = rect_loop((a.center[0], a.center[1]), (a.size[0], a.size[1]), a.z, a.speed, a.laps, a.corner)
    Path(a.output).write_text("# generated by `msim gen-trajectory rect`\n" + yaml.safe_dump(d, sort_keys=False, default_flow_style=None))
    print(f"wrote {a.output} ({len(d['waypoints'])} waypoints)")
    return 0


def _cmd_check(a) -> int:
    from .check import run_check
    rep = run_check(a.target, a.seconds, a.scans, a.imu_queue, a.scan_queue)
    print(rep.format())
    ok = (rep.imu.count > 1 and rep.scan.count > 1 and not rep.imu.error and not rep.scan.error
          and rep.imu.non_monotonic == 0 and rep.scan.non_monotonic == 0
          and rep.imu.gaps == 0 and rep.scan.gaps == 0)
    print("OK" if ok else "PROBLEMS FOUND")
    return 0 if ok else 1


def _cmd_compare(a) -> int:
    from .compare import compare, load_bench_trajectory, load_gt
    err = compare(load_gt(a.gt), load_bench_trajectory(a.estimate), a.offset_ns)
    print(err.summary())
    return 0


def _cmd_gen_pattern(a) -> int:
    import numpy as np
    from .pattern import DEFAULT_PATTERN_FILE, DEFAULT_RECORDING, build_recorded_pattern
    info = build_recorded_pattern(a.recording or DEFAULT_RECORDING, a.output or DEFAULT_PATTERN_FILE, a.max_scans)
    print(f"wrote {info['path']}: {info['scans']} scans, {100 * info['filled_fraction']:.1f} % of the directions "
          f"interpolated (misses), spin {np.mean(info['spin_deg_per_group']):.4f} deg per 4-point group")
    return 0


def _cmd_gen_proto(a) -> int:
    from ._proto import generate
    generate(force=True)
    print("generated stubs")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="msim", description=__doc__)
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    _add_serve(sub)
    p = sub.add_parser("gen-scene", help="write MJCF for a scene YAML")
    p.add_argument("scene")
    p.add_argument("-o", "--output")
    p.add_argument("--check", action="store_true", help="load the result in MuJoCo")
    p = sub.add_parser("gen-trajectory", help="write a scripted trajectory YAML")
    p.add_argument("shape", choices=["rect"])
    p.add_argument("--center", type=lambda s: tuple(float(x) for x in s.split(",")), default=(0.0, 0.0))
    p.add_argument("--size", type=lambda s: tuple(float(x) for x in s.split(",")), default=(3.0, 2.0),
                   help="width,height of the loop in m")
    p.add_argument("--z", type=float, default=1.0)
    p.add_argument("--speed", type=float, default=0.8)
    p.add_argument("--laps", type=int, default=1)
    p.add_argument("--corner", type=float, default=0.8, help="corner chamfer, m")
    p.add_argument("-o", "--output", required=True)
    p = sub.add_parser("check", help="connect to a running server and audit the streams")
    p.add_argument("target", nargs="?", default="localhost:50051")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--scans", type=int, help="stop after this many scans")
    p.add_argument("--imu-queue", type=int, default=1000)
    p.add_argument("--scan-queue", type=int, default=5)
    p = sub.add_parser("compare", help="compare a bench_slam --trajectory CSV with the ground-truth log")
    p.add_argument("gt")
    p.add_argument("estimate")
    p.add_argument("--offset-ns", type=int, default=47_995_000)
    p = sub.add_parser("gen-pattern", help="measure the Mid360 scan pattern of a .pbscan recording")
    p.add_argument("recording", nargs="?", help="default: ../test/data/living_room_garden.pbscan")
    p.add_argument("-o", "--output", help="default: out/mid360_pattern.npz (what --pattern measured loads)")
    p.add_argument("--max-scans", type=int)
    sub.add_parser("gen-proto", help="(re)generate the gRPC stubs from third_party/msensor/proto")

    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(name)s: %(message)s")
    handlers = {"serve": _cmd_serve, "gen-scene": _cmd_gen_scene, "gen-trajectory": _cmd_gen_traj,
                "check": _cmd_check, "compare": _cmd_compare, "gen-proto": _cmd_gen_proto,
                "gen-pattern": _cmd_gen_pattern}
    return handlers[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
