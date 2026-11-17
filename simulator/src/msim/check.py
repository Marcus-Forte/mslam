"""Client-side stream checker: connects like mslam does and audits what it receives."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import grpc
import numpy as np

from ._proto import ensure_generated

ensure_generated()
import imu_pb2  # noqa: E402
import imu_pb2_grpc  # noqa: E402
import lidar_pb2  # noqa: E402
import lidar_pb2_grpc  # noqa: E402


@dataclass
class StreamStats:
    count: int = 0
    first_ns: int = 0
    last_ns: int = 0
    gaps: int = 0           # sequence jumps (a drop on the server side)
    missing: int = 0        # samples lost according to the jumps
    non_monotonic: int = 0  # timestamp <= previous
    dts: list[int] = field(default_factory=list)
    ended_cleanly: bool = False
    error: str = ""

    def rate_hz(self) -> float:
        span = (self.last_ns - self.first_ns) * 1e-9
        return (self.count - 1) / span if self.count > 1 and span > 0 else 0.0


@dataclass
class CheckReport:
    imu: StreamStats = field(default_factory=StreamStats)
    scan: StreamStats = field(default_factory=StreamStats)
    wall_s: float = 0.0
    points: list[int] = field(default_factory=list)
    miss_fraction: float = 0.0
    min_hit_intensity: int = 0
    imu_before_scan_ns: int = 0
    last_imu_az_g: float = 0.0
    accel_min: list[float] = field(default_factory=lambda: [1e9] * 3)
    accel_max: list[float] = field(default_factory=lambda: [-1e9] * 3)
    accel_norm: list[float] = field(default_factory=lambda: [1e9, -1e9])   # min, max in g
    gyro_abs_max: list[float] = field(default_factory=lambda: [0.0] * 3)

    def format(self) -> str:
        def line(name: str, s: StreamStats, spacing_ms: float) -> str:
            dts = np.array(s.dts) * 1e-6 if s.dts else np.zeros(1)
            return (f"{name}: {s.count} msgs, sim rate {s.rate_hz():.2f} Hz, dt min/mean/max "
                    f"{dts.min():.3f}/{dts.mean():.3f}/{dts.max():.3f} ms (nominal {spacing_ms}), "
                    f"seq gaps {s.gaps} ({s.missing} lost), non-monotonic {s.non_monotonic}, "
                    f"{'clean end' if s.ended_cleanly else 'open'}{' ERROR ' + s.error if s.error else ''}")
        pts = self.points or [0]
        out = [line("IMU ", self.imu, 5.0), line("Scan", self.scan, 48.0),
               f"points per scan min/max {min(pts)}/{max(pts)}, misses {100 * self.miss_fraction:.1f} %, "
               f"min hit intensity {self.min_hit_intensity}",
               f"wall time {self.wall_s:.2f} s -> wall rates IMU {self.imu.count / max(self.wall_s, 1e-9):.1f} Hz, "
               f"scan {self.scan.count / max(self.wall_s, 1e-9):.1f} Hz; "
               f"last IMU az {self.last_imu_az_g:+.3f} g",
               "accel g min " + " ".join(f"{v:+.2f}" for v in self.accel_min) + " max "
               + " ".join(f"{v:+.2f}" for v in self.accel_max) + f", |a| {self.accel_norm[0]:.3f}..{self.accel_norm[1]:.3f}"
               + ", |gyro| max rad/s " + " ".join(f"{v:.2f}" for v in self.gyro_abs_max)]
        return "\n".join(out)


def _track(st: StreamStats, ts: int, seq: int, last_seq: list[int | None]) -> None:
    if st.count:
        if ts <= st.last_ns:
            st.non_monotonic += 1
        st.dts.append(ts - st.last_ns)
        jump = (seq - last_seq[0]) & 0xFFFFFFFF
        if jump != 1:
            st.gaps += 1
            st.missing += max(jump - 1, 0)
    else:
        st.first_ns = ts
    st.last_ns = ts
    last_seq[0] = seq
    st.count += 1


def run_check(target: str, seconds: float = 5.0, max_scans: int | None = None,
              imu_queue: int = 1000, scan_queue: int = 5) -> CheckReport:
    """Subscribe to both streams for `seconds` of wall time (or until `max_scans` scans / end of stream)."""
    rep = CheckReport()
    stop = threading.Event()
    channel = grpc.insecure_channel(target, options=[("grpc.max_receive_message_length", 64 * 1024 * 1024)])
    miss = [0, 0]
    min_int = [10**9]
    lock = threading.Lock()

    def imu_worker() -> None:
        last: list[int | None] = [None]
        try:
            call = imu_pb2_grpc.ImuServiceStub(channel).getImuData(imu_pb2.ImuStreamRequest(max_queued_samples=imu_queue))
            for m in call:
                _track(rep.imu, m.header.timestamp, m.header.sequence_number, last)
                rep.last_imu_az_g = m.az
                acc = (m.ax, m.ay, m.az)
                for i in range(3):
                    rep.accel_min[i] = min(rep.accel_min[i], acc[i])
                    rep.accel_max[i] = max(rep.accel_max[i], acc[i])
                    rep.gyro_abs_max[i] = max(rep.gyro_abs_max[i], abs((m.gx, m.gy, m.gz)[i]))
                n = (acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2) ** 0.5
                rep.accel_norm = [min(rep.accel_norm[0], n), max(rep.accel_norm[1], n)]
                if stop.is_set():
                    call.cancel()
                    return
            rep.imu.ended_cleanly = True
        except grpc.RpcError as e:
            if e.code() != grpc.StatusCode.CANCELLED:
                rep.imu.error = f"{e.code().name}: {e.details()}"

    def scan_worker() -> None:
        last: list[int | None] = [None]
        try:
            call = lidar_pb2_grpc.LidarServiceStub(channel).getLidarScan(lidar_pb2.LidarStreamRequest(max_queued_samples=scan_queue))
            for m in call:
                with lock:
                    if rep.scan.count == 0:
                        rep.imu_before_scan_ns = rep.imu.first_ns
                    _track(rep.scan, m.header.timestamp, m.header.sequence_number, last)
                    rep.points.append(len(m.x))
                    inten = np.asarray(m.intensity)
                    hit = inten > 0
                    miss[0] += int((~hit).sum())
                    miss[1] += len(inten)
                    if hit.any():
                        min_int[0] = min(min_int[0], int(inten[hit].min()))
                if stop.is_set() or (max_scans is not None and rep.scan.count >= max_scans):
                    call.cancel()
                    return
            rep.scan.ended_cleanly = True
        except grpc.RpcError as e:
            if e.code() != grpc.StatusCode.CANCELLED:
                rep.scan.error = f"{e.code().name}: {e.details()}"

    t0 = time.perf_counter()
    threads = [threading.Thread(target=imu_worker, daemon=True), threading.Thread(target=scan_worker, daemon=True)]
    for t in threads:
        t.start()
    deadline = t0 + seconds
    while time.perf_counter() < deadline and any(t.is_alive() for t in threads):
        if max_scans is not None and rep.scan.count >= max_scans:
            break
        time.sleep(0.02)
    stop.set()
    rep.wall_s = time.perf_counter() - t0
    channel.close()
    for t in threads:
        t.join(timeout=3.0)
    rep.miss_fraction = miss[0] / miss[1] if miss[1] else 0.0
    rep.min_hit_intensity = min_int[0] if min_int[0] != 10**9 else 0
    return rep
