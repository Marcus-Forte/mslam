#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Compare the two IMU fusion modes from two bench_slam --trajectory CSVs.

    uv run test/plot_imu_fusion.py A.csv B.csv out.png

A is the run with the preintegration factor, B the dead-reckoning run (both
from the same recording). Panels: XY trajectories, per-scan first-round
correction (how far the prior was from the registered pose), speed, and the
gyro and accelerometer bias norms. Time is seconds from the first scan.
"""
import argparse
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Colour-blind safe pair (Okabe-Ito blue and vermillion); the line styles
# differ too, so the two stay distinguishable in greyscale.
STYLES = {
    "A": dict(color="#0072B2", ls="-", label="A: preintegration factor"),
    "B": dict(color="#D55E00", ls="--", label="B: dead reckoning"),
}


def load(path: str) -> dict[str, np.ndarray]:
    # Empty fields (first round without correspondences) become NaN.
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        sys.exit(f"{path}: no rows")
    return {name: np.atleast_1d(data[name]) for name in data.dtype.names}


def norm(d: dict[str, np.ndarray], *names: str) -> np.ndarray:
    return np.linalg.norm(np.stack([d[n] for n in names]), axis=0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("preintegration_csv", help="run A")
    parser.add_argument("dead_reckoning_csv", help="run B")
    parser.add_argument("output", help="output image (.png, .svg, ...)")
    parser.add_argument("--title", default="IMU fusion: preintegration factor vs dead reckoning")
    args = parser.parse_args()

    runs = {"A": load(args.preintegration_csv), "B": load(args.dead_reckoning_csv)}
    for run in runs.values():
        run["t"] = (run["timestamp_ns"] - run["timestamp_ns"][0]) * 1e-9

    fig, ax = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    fig.suptitle(args.title)

    # B is drawn first and wider, A on top and thinner, so neither hides the
    # other where they coincide (the trajectories and the speed do).
    widths = {"A": 1.0, "B": 2.0}

    def line(a, key, x, y, **kw):
        a.plot(x, y, color=STYLES[key]["color"], ls=STYLES[key]["ls"],
               label=STYLES[key]["label"], lw=widths[key], zorder=3 if key == "A" else 2, **kw)

    # XY trajectory. Equal aspect so distances are true.
    a = ax[0, 0]
    for key, d in sorted(runs.items(), reverse=True):
        line(a, key, d["x"], d["y"])
        a.plot(d["x"][-1], d["y"][-1], "o", color=STYLES[key]["color"], ms=6)
    a.plot(0, 0, "k*", ms=11, label="start")
    a.set_xlabel("x (m)")
    a.set_ylabel("y (m)")
    a.set_title("Trajectory, top view (dot = final position)")
    a.set_aspect("equal", adjustable="datalim")
    a.grid(alpha=0.3)
    a.legend()

    def over_time(a, transform, ylabel, title):
        for key, d in sorted(runs.items(), reverse=True):
            line(a, key, d["t"], transform(d))
        a.set_xlabel("time (s)")
        a.set_ylabel(ylabel)
        a.set_title(title)
        a.grid(alpha=0.3)
        a.legend()

    over_time(ax[0, 1], lambda d: d["first_round_mm"], "correction (mm)",
              "First-round translation correction (lower = better prior)")
    over_time(ax[1, 0], lambda d: d["first_round_mrad"], "correction (mrad)",
              "First-round rotation correction (lower = better prior)")
    over_time(ax[1, 1], lambda d: norm(d, "vx", "vy", "vz"),
              "speed |v| (m/s)", "Speed")
    over_time(ax[2, 0], lambda d: norm(d, "bgx", "bgy", "bgz"),
              "|gyro bias| (rad/s)", "Gyro bias estimate (B never estimates it)")
    over_time(ax[2, 1], lambda d: norm(d, "bax", "bay", "baz"),
              "|accel bias| (m/s^2)", "Accelerometer bias estimate (B never estimates it)")

    fig.savefig(args.output, dpi=150)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
