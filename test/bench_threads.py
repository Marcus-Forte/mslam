#!/usr/bin/env python3
"""Compare bench_slam at several thread counts (and builds): interleaved runs, minima, pose agreement.

Each --run is LABEL=BINARY, a bench_slam binary. It is run with `--threads N` for every --threads
value, so one build gives one row per thread count, and several builds (say a reference binary and
a new one) can be compared side by side:

  test/bench_threads.py \\
      --run new=build/gcc/test/bench_slam \\
      --threads 1 2 4 0 --rounds 5 test/data/living_room_garden.pbscan test/data/rooms.pbscan

For every recording the runs go round by round, each round over every (label, threads) pair in a
rotated order, so that a slow phase of the machine does not land on one row. The table gives each
row's minimum total, register, search, normals and solve times, the ratio of its total to the
reference row (--ref, LABEL or LABEL:THREADS; the first label at the first --threads value by
default), its final pose and how far that pose is from the reference's (translation in mm, largest
rotation component in mrad). --threads 0 is every hardware thread; a bench_slam built without
oneTBB runs sequentially at any value.

Exit status 1 when a run fails, when a row's pose differs between rounds or when a label's pose
differs between thread counts (the pipeline is deterministic and the thread count must not change
the pose, bit for bit: that is a bug, not noise), or when a pose is further from the reference's
than --tolerance-mm / --tolerance-mrad (0 by default: poses of one commit must be identical;
raise it to compare builds that are expected to differ in the last digits). --history FILE appends
one JSON line per row and recording (date, commit, times, pose), to follow the timings over time.
"""
import argparse
import datetime
import json
import math
import re
import subprocess
import sys
from pathlib import Path

RUN = re.compile(
    r"^run (?P<run>\d+) scans=(?P<scans>\d+) total=(?P<total>[\d.]+) ms \| preprocess=(?P<preprocess>[\d.]+) "
    r"register=(?P<register>[\d.]+) map_insert=(?P<map_insert>[\d.]+) predict=(?P<predict>[\d.]+) \| "
    r"pose=\[(?P<pose>[^\]]+)\]"
)
SPLIT = re.compile(
    r"register split: search=(?P<search>[\d.]+) normals=(?P<normals>[\d.]+) assembly=(?P<assembly>[\d.]+) "
    r"solve=(?P<solve>[\d.]+) other=(?P<other>-?[\d.]+) ms"
)
MAP_POSE = re.compile(r"pose map =\[(?P<pose>[^\]]+)\]")


def parse_run_spec(text):
    label, sep, binary = text.partition("=")
    if not sep or not label or not binary:
        raise argparse.ArgumentTypeError(f"--run wants LABEL=BINARY, got {text!r}")
    return label, Path(binary)


def parse_output(text):
    """The runs of one bench_slam invocation: [{total, register, solve, ..., pose}]."""
    runs = []
    for line in text.splitlines():
        if match := RUN.match(line):
            runs.append({key: float(match[key]) for key in ("total", "preprocess", "register", "map_insert")}
                        | {"scans": int(match["scans"]), "pose": match["pose"].split()})
        elif (match := SPLIT.search(line)) and runs:
            runs[-1] |= {key: float(match[key]) for key in ("search", "normals", "assembly", "solve")}
        elif (match := MAP_POSE.search(line)) and runs:
            runs[-1]["map_pose"] = match["pose"].split()
    return runs


def pose_distance(a, b):
    """(translation distance in mm, largest rotation difference in mrad) between two pose strings."""
    pa, pb = [float(v) for v in a], [float(v) for v in b]
    translation = math.dist(pa[:3], pb[:3]) * 1e3
    rotation = max(abs(math.remainder(x - y, 2 * math.pi)) for x, y in zip(pa[3:], pb[3:])) * 1e3
    return translation, rotation


def git_state(path):
    try:
        top = subprocess.run(["git", "-C", str(path), "rev-parse", "--short", "HEAD"], capture_output=True,
                             text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(path), "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True, check=True).stdout.strip()
        return top + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recordings", nargs="+", type=Path, help=".pbscan recordings")
    parser.add_argument("--run", dest="runs", action="append", type=parse_run_spec, required=True,
                        metavar="LABEL=BINARY", help="a bench_slam binary")
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "integration" / "mslam.jsonc")
    parser.add_argument("--threads", nargs="+", type=int, default=[1, 4], help="--threads values (default 1 4)")
    parser.add_argument("--rounds", type=int, default=3, help="interleaved rounds (default 3)")
    parser.add_argument("--repeats", type=int, default=1, help="bench_slam repeats per invocation (default 1)")
    parser.add_argument("--strategy", choices=["odometry", "pose_graph"], help="override slam.strategy")
    parser.add_argument("--ref", help="reference row: LABEL or LABEL:THREADS (default: the first label at the "
                                      "first --threads value)")
    parser.add_argument("--tolerance-mm", type=float, default=0.0, help="pose translation tolerance (default 0)")
    parser.add_argument("--tolerance-mrad", type=float, default=0.0, help="pose rotation tolerance (default 0)")
    parser.add_argument("--out", type=Path, help="keep every bench_slam output in this directory")
    parser.add_argument("--history", type=Path, help="append the results to this JSON-lines file")
    args = parser.parse_args()

    labels = [label for label, _ in args.runs]
    if len(set(labels)) != len(labels):
        parser.error("--run labels must be unique")
    for label, binary in args.runs:
        if not binary.is_file():
            parser.error(f"{label}: {binary} does not exist")
    rows = [(label, binary, threads) for label, binary in args.runs for threads in args.threads]
    ref_label, _, ref_threads = (args.ref or f"{labels[0]}:{args.threads[0]}").partition(":")
    ref_threads = int(ref_threads) if ref_threads else args.threads[0]
    if ref_label not in labels or ref_threads not in args.threads:
        parser.error(f"--ref {args.ref} is not a --run label with a --threads value")
    ref = (ref_label, ref_threads)
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)

    failures = []
    # results[recording][(label, threads)] = list of runs (dicts), over rounds and repeats
    results = {}
    for recording in args.recordings:
        by_row = results[recording.stem] = {(label, threads): [] for label, _, threads in rows}
        for round_index in range(args.rounds):
            shift = round_index % len(rows)
            for label, binary, threads in rows[shift:] + rows[:shift]:
                command = [str(binary), str(args.config), str(recording), str(args.repeats), "--threads",
                           str(threads)]
                if args.strategy:
                    command += ["--strategy", args.strategy]
                done = subprocess.run(command, capture_output=True, text=True)
                if args.out:
                    name = f"{label}.{recording.stem}.t{threads}.{round_index}.txt"
                    (args.out / name).write_text(done.stdout + done.stderr)
                runs = parse_output(done.stdout)
                if done.returncode != 0 or not runs:
                    failures.append(f"{label} {recording.stem} t{threads}: exit {done.returncode}: "
                                    f"{(done.stderr or done.stdout).strip().splitlines()[-1:]}")
                    continue
                by_row[(label, threads)].extend(runs)
            print(f"{recording.stem}: round {round_index + 1}/{args.rounds} done", file=sys.stderr)

    history = []
    stamp = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    commit = git_state(Path(__file__).resolve().parent)
    final_pose = {}  # (label, recording) -> {threads: pose}
    for recording, by_row in results.items():
        print(f"\n## {recording} ({args.rounds} rounds x {args.repeats})\n")
        print("| label | threads | total ms | ratio | register ms | search ms | normals ms | solve ms | pose "
              "| vs ref |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        ref_runs = by_row.get(ref) or []
        ref_total = min((run["total"] for run in ref_runs), default=None)
        ref_pose = ref_runs[0]["pose"] if ref_runs else None
        for label, _, threads in rows:
            runs = by_row[(label, threads)]
            if not runs:
                print(f"| {label} | {threads} | failed | | | | | | | |")
                continue
            poses = {" ".join(run["pose"]) for run in runs}
            if len(poses) > 1:
                failures.append(f"{label} {recording} t{threads}: the pose differs between runs: {sorted(poses)}")
            pose = runs[0]["pose"]
            final_pose.setdefault((label, recording), {})[threads] = pose
            best = {key: min(run[key] for run in runs if key in run)
                    for key in ("total", "register", "search", "normals", "solve") if any(key in run for run in runs)}
            cell = lambda key: f"{best[key]:.1f}" if key in best else "-"
            ratio = f"{best['total'] / ref_total:.3f}" if ref_total else "-"
            if ref_pose is None or (label, threads) == ref:
                versus = "(ref)" if (label, threads) == ref else "-"
            else:
                mm, mrad = pose_distance(pose, ref_pose)
                flag = "!" if mm > args.tolerance_mm or mrad > args.tolerance_mrad else ""
                if flag:
                    failures.append(f"{label} {recording} t{threads}: pose {mm:.3f} mm / {mrad:.3f} mrad from "
                                    f"{ref_label}:{ref_threads}")
                versus = f"{mm:.3f} mm {mrad:.3f} mrad{flag}"
            print(f"| {label} | {threads} | {cell('total')} | {ratio} | {cell('register')} | {cell('search')} | "
                  f"{cell('normals')} | {cell('solve')} | {' '.join(pose)} | {versus} |")
            history.append({"date": stamp, "commit": commit, "label": label, "recording": recording,
                            "threads": threads, "rounds": len(runs), "scans": runs[0]["scans"],
                            "min_ms": best, "pose": pose})

    for (label, recording), by_threads in final_pose.items():
        if len({" ".join(pose) for pose in by_threads.values()}) > 1:
            failures.append(f"{label} {recording}: the pose differs between thread counts {sorted(by_threads)}")

    if args.history:
        with args.history.open("a") as stream:
            for row in history:
                stream.write(json.dumps(row) + "\n")

    if failures:
        print("\nFAILED:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(f"\nOK: every pose is stable across rounds and thread counts and within "
          f"{args.tolerance_mm} mm / {args.tolerance_mrad} mrad of {ref_label}:{ref_threads}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
