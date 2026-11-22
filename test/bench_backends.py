#!/usr/bin/env python3
"""Compare bench_slam on several builds and paths: interleaved runs, minima, pose agreement.

Each --run is LABEL=BINARY[:host|sycl|none], a bench_slam binary run with `--no-sycl` (host, the
default when omitted), `--sycl` (sycl) or neither (`none`, for a bench_slam older than those options).
--threads is passed as `--threads N`: the work-groups of the SYCL kernels in a current binary (no
effect on the host path), the thread count of an older binary. One build with SYCL can appear twice
(its host and its sycl path), next to a host-only build or an older reference binary:

  test/bench_backends.py \\
      --run host=build/gcc/test/bench_slam:host \\
      --run sycl=build/sycl/test/bench_slam:sycl \\
      --run sycl-host=build/sycl/test/bench_slam:host \\
      --threads 1 4 --rounds 5 test/data/living_room_garden.pbscan test/data/rooms.pbscan

For every recording and thread count the runs go round by round, each round in a rotated label
order, so that a slow phase of the machine does not land on one label. The table gives each label's
minimum total, register and solve times, the ratio of its total to the reference label (--ref, the
first --run by default), its moptim launches, its final pose and how far that pose is from the
reference's (translation in mm, largest rotation component in mrad).

Exit status 1 when a run fails, when a label's pose differs between rounds or between thread counts
(the pipeline is deterministic: that is a bug, not noise), or when a pose is further from the
reference's than --tolerance-mm / --tolerance-mrad. --history FILE appends one JSON line per label,
recording and thread count (date, commit, times, pose), to follow the backends over time.
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
    r"solve=(?P<solve>[\d.]+) other=(?P<other>-?[\d.]+) ms \| moptim_launches=(?P<launches>\d+) "
    r"device_search_launches=(?P<device_launches>\d+)"
)
MAP_POSE = re.compile(r"pose map =\[(?P<pose>[^\]]+)\]")
BACKEND = re.compile(r"^backend: (?P<backend>\S+) \(built with SYCL: (?P<sycl>\w+)\)")


def parse_run_spec(text):
    label, sep, rest = text.partition("=")
    if not sep or not label or not rest:
        raise argparse.ArgumentTypeError(f"--run wants LABEL=BINARY[:host|sycl|none], got {text!r}")
    binary, _, backend = rest.rpartition(":") if rest.endswith((":host", ":sycl", ":none")) else (rest, "", "host")
    return label, Path(binary), backend


def parse_output(text):
    """The runs of one bench_slam invocation: [{total, register, solve, ..., pose, launches}]."""
    runs, backend = [], None
    for line in text.splitlines():
        if match := BACKEND.match(line):
            backend = match["backend"]
        elif match := RUN.match(line):
            runs.append({key: float(match[key]) for key in ("total", "preprocess", "register", "map_insert")}
                        | {"scans": int(match["scans"]), "pose": match["pose"].split()})
        elif (match := SPLIT.search(line)) and runs:
            runs[-1] |= {key: float(match[key]) for key in ("search", "normals", "assembly", "solve")}
            runs[-1] |= {"launches": int(match["launches"]), "device_launches": int(match["device_launches"])}
        elif (match := MAP_POSE.search(line)) and runs:
            runs[-1]["map_pose"] = match["pose"].split()
    return backend, runs


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
                        metavar="LABEL=BINARY[:host|sycl|none]", help="a bench_slam binary and the path it runs")
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "integration" / "mslam.jsonc")
    parser.add_argument("--threads", nargs="+", type=int, default=[1], help="--threads values (default 1)")
    parser.add_argument("--rounds", type=int, default=3, help="interleaved rounds (default 3)")
    parser.add_argument("--repeats", type=int, default=1, help="bench_slam repeats per invocation (default 1)")
    parser.add_argument("--strategy", choices=["odometry", "pose_graph"], help="override slam.strategy")
    parser.add_argument("--ref", help="reference label (default: the first --run)")
    parser.add_argument("--tolerance-mm", type=float, default=0.5, help="pose translation tolerance (default 0.5)")
    parser.add_argument("--tolerance-mrad", type=float, default=0.15, help="pose rotation tolerance (default 0.15)")
    parser.add_argument("--out", type=Path, help="keep every bench_slam output in this directory")
    parser.add_argument("--history", type=Path, help="append the results to this JSON-lines file")
    args = parser.parse_args()

    labels = [label for label, _, _ in args.runs]
    if len(set(labels)) != len(labels):
        parser.error("--run labels must be unique")
    ref = args.ref or labels[0]
    if ref not in labels:
        parser.error(f"--ref {ref} is not a --run label")
    for label, binary, backend in args.runs:
        if not binary.is_file():
            parser.error(f"{label}: {binary} does not exist")
        if backend not in ("host", "sycl", "none"):
            parser.error(f"{label}: backend must be host, sycl or none")
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)

    failures = []
    # results[(recording, threads)][label] = list of runs (dicts), over rounds and repeats
    results = {}
    for recording in args.recordings:
        for threads in args.threads:
            key = (recording.stem, threads)
            results[key] = {label: [] for label in labels}
            for round_index in range(args.rounds):
                shift = round_index % len(args.runs)
                for label, binary, backend in args.runs[shift:] + args.runs[:shift]:
                    command = [str(binary), str(args.config), str(recording), str(args.repeats), "--threads",
                               str(threads)]
                    if backend != "none":
                        command += ["--sycl" if backend == "sycl" else "--no-sycl"]
                    if args.strategy:
                        command += ["--strategy", args.strategy]
                    done = subprocess.run(command, capture_output=True, text=True)
                    if args.out:
                        name = f"{label}.{recording.stem}.t{threads}.{round_index}.txt"
                        (args.out / name).write_text(done.stdout + done.stderr)
                    printed_backend, runs = parse_output(done.stdout)
                    if done.returncode != 0 or not runs:
                        failures.append(f"{label} {recording.stem} t{threads}: exit {done.returncode}: "
                                        f"{(done.stderr or done.stdout).strip().splitlines()[-1:]}")
                        continue
                    if backend != "none" and printed_backend != backend:
                        failures.append(f"{label}: ran on backend {printed_backend}, not {backend}")
                    results[key][label].extend(runs)
                print(f"{recording.stem} t{threads}: round {round_index + 1}/{args.rounds} done", file=sys.stderr)

    history = []
    stamp = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    commit = git_state(Path(__file__).resolve().parent)
    final_pose = {}  # (label, recording) -> {threads: pose}
    for (recording, threads), by_label in results.items():
        print(f"\n## {recording}, --threads {threads} ({args.rounds} rounds x {args.repeats})\n")
        print("| label | total ms | ratio | register ms | search ms | normals ms | solve ms | moptim launches | pose "
              "| vs ref |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        ref_runs = by_label.get(ref) or []
        ref_total = min((run["total"] for run in ref_runs), default=None)
        ref_pose = ref_runs[0]["pose"] if ref_runs else None
        for label in labels:
            runs = by_label[label]
            if not runs:
                print(f"| {label} | failed | | | | | | | | |")
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
            if ref_pose is None or label == ref:
                versus = "(ref)" if label == ref else "-"
            else:
                mm, mrad = pose_distance(pose, ref_pose)
                flag = "!" if mm > args.tolerance_mm or mrad > args.tolerance_mrad else ""
                if flag:
                    failures.append(f"{label} {recording} t{threads}: pose {mm:.3f} mm / {mrad:.3f} mrad from {ref}")
                versus = f"{mm:.3f} mm {mrad:.3f} mrad{flag}"
            print(f"| {label} | {cell('total')} | {ratio} | {cell('register')} | {cell('search')} | {cell('normals')} | "
                  f"{cell('solve')} | {runs[0].get('launches', '-')} | {' '.join(pose)} | {versus} |")
            history.append({"date": stamp, "commit": commit, "label": label, "recording": recording,
                            "threads": threads, "backend": dict((l, b) for l, _, b in args.runs)[label],
                            "rounds": len(runs), "scans": runs[0]["scans"], "launches": runs[0].get("launches"),
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
          f"{args.tolerance_mm} mm / {args.tolerance_mrad} mrad of {ref}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
