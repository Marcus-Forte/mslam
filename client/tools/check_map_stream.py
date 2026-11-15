"""Headless end-to-end check of the map stream against a running mslam (no viser, no browser).

Three consumers, each a `MapStore` fed by `stream_map`:
  A  connects at once and reads to the end;
  B  connects when A has reached `--half-offset` (about half of the run) and catches up;
  C  connects at once, disconnects at `--disconnect-offset`, waits `--gap-sec` and resumes with its stored
     `(epoch, next_offset)`.
When the run is over (no consumer advanced for `--idle-sec`) it prints epoch, final next_offset, point count and a
checksum per consumer and requires them to agree. Then it calls `Reset` and requires every consumer to see a new
epoch and an empty map.

Start playback (for example with Space on the playback publisher's stdin) after this script printed "ready".
Exit status 0 means every check passed.
"""

import argparse
import hashlib
import logging
import sys
import threading
import time
from pathlib import Path

import grpc

CLIENT_DIR = Path(__file__).resolve().parent.parent
if str(CLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(CLIENT_DIR))

from map_store import MapStore  # noqa: E402
from map_stream import StreamEnd, stream_map  # noqa: E402
from proto_gen import slam_pb2, slam_pb2_grpc  # noqa: E402

logger = logging.getLogger("check_map_stream")


class Consumer:
    def __init__(self, name: str, server: str, disconnect_offset: int | None = None, gap_sec: float = 0.0) -> None:
        self.name = name
        self.store = MapStore()
        self.connections = 0
        self.chunks = 0
        self.last_progress = time.monotonic()
        self.epochs_seen: list[int] = []
        self._server = server
        self._disconnect_offset = disconnect_offset
        self._gap_sec = gap_sec
        self._disconnected_once = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _should_stop(self) -> bool:
        if self._stop.is_set():
            return True
        return (
            self._disconnect_offset is not None
            and not self._disconnected_once
            and self.store.resume_position.next_offset >= self._disconnect_offset
        )

    def _on_change(self, _change: object) -> None:
        self.chunks += 1
        self.last_progress = time.monotonic()
        epoch = self.store.resume_position.epoch
        if not self.epochs_seen or self.epochs_seen[-1] != epoch:
            self.epochs_seen.append(epoch)

    def _run(self) -> None:
        while not self._stop.is_set():
            self.connections += 1
            try:
                with grpc.insecure_channel(self._server) as channel:
                    end = stream_map(
                        slam_pb2_grpc.SlamServiceStub(channel),
                        self.store,
                        on_change=self._on_change,
                        should_stop=self._should_stop,
                    )
            except grpc.RpcError as exc:
                logger.error("%s: stream error %s", self.name, exc)
                self._stop.wait(1.0)
                continue
            if end is StreamEnd.STOPPED and not self._stop.is_set():
                self._disconnected_once = True
                logger.info(
                    "%s: disconnected at %s, resuming in %.1fs", self.name, self.store.resume_position, self._gap_sec
                )
                self._stop.wait(self._gap_sec)

    def summary(self) -> dict[str, object]:
        epoch, next_offset = self.store.resume_position
        points = self.store.all_points()
        colors = self.store.all_colors()
        return {
            "epoch": epoch,
            "next_offset": next_offset,
            "points": len(points),
            "points_sha256": hashlib.sha256(points.tobytes()).hexdigest()[:16],
            "colors_sha256": hashlib.sha256(colors.tobytes()).hexdigest()[:16],
            "connections": self.connections,
            "chunks": self.chunks,
        }


def report(consumers: list[Consumer], title: str) -> list[dict[str, object]]:
    print(f"--- {title}")
    summaries = []
    for consumer in consumers:
        summary = consumer.summary()
        summaries.append(summary)
        print(f"{consumer.name}: " + " ".join(f"{k}={v}" for k, v in summary.items()))
    return summaries


def wait_for(condition, timeout: float, what: str) -> bool:  # noqa: ANN001
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    print(f"timeout waiting for {what}")
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default="127.0.0.1:50052")
    parser.add_argument("--half-offset", type=int, default=500, help="B connects when A reaches this offset")
    parser.add_argument("--disconnect-offset", type=int, default=200, help="C disconnects at this offset")
    parser.add_argument("--gap-sec", type=float, default=3.0, help="how long C stays disconnected")
    parser.add_argument("--idle-sec", type=float, default=6.0, help="no progress for this long means the run is over")
    parser.add_argument("--start-timeout", type=float, default=120.0)
    parser.add_argument("--run-timeout", type=float, default=600.0)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    a = Consumer("A", args.server)
    b = Consumer("B", args.server)
    c = Consumer("C", args.server, args.disconnect_offset, args.gap_sec)
    a.start()
    c.start()
    print("ready: start playback now", flush=True)

    if not wait_for(
        lambda: a.store.resume_position.next_offset >= args.half_offset, args.run_timeout, "A to reach half"
    ):
        return 2
    print(f"B connects now (A at {a.store.resume_position})", flush=True)
    b.start()

    def finished() -> bool:
        now = time.monotonic()
        positions = {x.store.resume_position for x in (a, b, c)}
        return (
            a.store.resume_position.next_offset > 0
            and len(positions) == 1
            and all(now - x.last_progress > args.idle_sec for x in (a, b, c))
        )

    if not wait_for(finished, args.run_timeout, "the run to end with all consumers equal"):
        report([a, b, c], "state at timeout")
        return 2

    consumers = [a, b, c]
    summaries = report(consumers, "end of run")
    ok = True
    for key in ("epoch", "next_offset", "points", "points_sha256", "colors_sha256"):
        if len({s[key] for s in summaries}) != 1:
            print(f"FAIL: consumers disagree on {key}")
            ok = False
    if summaries[0]["points"] == 0:
        print("FAIL: the map is empty")
        ok = False
    if c.connections < 2:
        print("FAIL: C did not reconnect")
        ok = False
    print(f"C connections={c.connections}, B started at A offset >= {args.half_offset}")
    old_epoch = summaries[0]["epoch"]

    with grpc.insecure_channel(args.server) as channel:
        slam_pb2_grpc.SlamServiceStub(channel).Reset(slam_pb2.Empty())
    print("Reset sent")

    def reset_seen() -> bool:
        return all(x.store.resume_position.epoch not in (0, old_epoch) for x in consumers)

    if not wait_for(reset_seen, 30.0, "all consumers to see the new epoch"):
        ok = False
    time.sleep(1.0)
    after = report(consumers, "after Reset")
    if len({s["epoch"] for s in after}) != 1 or after[0]["epoch"] == old_epoch:
        print("FAIL: consumers do not share a new epoch")
        ok = False
    if any(s["points"] != 0 or s["next_offset"] != 0 for s in after):
        print("FAIL: map not empty at offset 0 after Reset")
        ok = False

    for consumer in consumers:
        consumer.stop()
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
