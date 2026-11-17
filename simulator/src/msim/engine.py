"""Simulation engine: one clock, one trajectory, both sensor streams.

Time is a nanosecond integer clock starting at `start_ns` (tick 0). The kinematic
body is stepped every 1 ms into the pose history; IMU samples (every 5 ms) and
LiDAR scans (every 48 ms, 9600 points 5 us apart) are produced from that history
in timestamp order as soon as the clock has passed them. A scan is published when
its last point has been "acquired", with `header.timestamp` = its FIRST point,
like the real driver.

Pacing: `speed > 0` sleeps so that simulated time follows wall time x speed;
`speed == 0` runs as fast as the sinks accept data (deterministic; use the
lossless `block` back-pressure of the hub so nothing is dropped).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .geometry import quat_from_rot
from .history import TICK_NS, PoseHistory
from .motion import Kin, MotionSource
from .sensors import ImuModel, ImuSample, LidarModel, ScanData

log = logging.getLogger("msim.engine")


@dataclass
class GroundTruth:
    t_ns: int
    seq: int
    p: np.ndarray   # (3,)
    q: np.ndarray   # (4,) w,x,y,z
    R: np.ndarray   # (3,3)


@dataclass
class EngineConfig:
    start_ns: int = 1_000_000_000_000
    imu_period_ns: int = 5_000_000
    scan_period_ns: int = 48_000_000
    imu_phase_ns: int = 1_300_000   # IMU is not phase-locked to the scans
    scan_phase_ns: int = 0
    speed: float = 1.0              # 0 = unpaced
    history_capacity: int = 8192


def kin_to_history_row(k: Kin):
    return k.p, quat_from_rot(k.R), k.v, k.a, k.w


class SimEngine:
    def __init__(self, motion: MotionSource, lidar: LidarModel | None, imu: ImuModel,
                 config: EngineConfig | None = None) -> None:
        self.cfg = config or EngineConfig()
        self.motion, self.lidar, self.imu = motion, lidar, imu
        self.history = PoseHistory(self.cfg.start_ns, self.cfg.history_capacity)
        self.imu_sinks: list[Callable[[ImuSample], None]] = []
        self.scan_sinks: list[Callable[[ScanData, GroundTruth], None]] = []
        self.scan_wanted: Callable[[], bool] = lambda: bool(self.scan_sinks)
        self.start_gate: Callable[[threading.Event], None] | None = None
        self.on_finish: list[Callable[[], None]] = []
        self.stop_event = threading.Event()
        self.done = threading.Event()
        self.latest: Kin | None = None   # for the viewer (atomic reference swap)
        self.latest_t_ns = self.cfg.start_ns
        self._imu_seq = 0
        self._scan_seq = 0

    # -- helpers -----------------------------------------------------------
    def _append(self, k: Kin) -> None:
        self.history.append(*kin_to_history_row(k))
        self.latest = k
        self.latest_t_ns = self.history.latest_ns

    def run(self) -> None:
        try:
            self._run()
        finally:
            self.done.set()
            for cb in self.on_finish:
                cb()

    def _run(self) -> None:
        cfg = self.cfg
        if self.start_gate is not None:
            self.start_gate(self.stop_event)
        if self.stop_event.is_set():
            return
        self._append(self.motion.initial())
        n_imu = 0
        scan_idx = 0
        n_pts = self.lidar.points_per_scan if self.lidar else 0
        span_ns = int(np.rint((n_pts - 1) * 1e9 / self.lidar.points_per_second)) if self.lidar else 0
        wall0 = time.perf_counter()
        k = 0
        warned = False
        while not self.stop_event.is_set():
            # produce everything that is due at the current history end
            now_ns = self.history.latest_ns
            while True:
                t_imu = cfg.start_ns + cfg.imu_phase_ns + n_imu * cfg.imu_period_ns
                t_scan0 = cfg.start_ns + cfg.scan_phase_ns + scan_idx * cfg.scan_period_ns
                t_scan_end = t_scan0 + span_ns
                due_imu = t_imu <= now_ns
                due_scan = self.lidar is not None and t_scan_end <= now_ns
                if not (due_imu or due_scan):
                    break
                if due_imu and (not due_scan or t_imu <= t_scan_end):
                    sample = self.imu.sample(self.history, t_imu, self._imu_seq)
                    self._imu_seq += 1
                    n_imu += 1
                    for s in self.imu_sinks:
                        s(sample)
                else:
                    if self.scan_wanted():
                        scan = self.lidar.scan(self.history, t_scan0, scan_idx)
                        scan.seq = self._scan_seq
                        self._scan_seq += 1
                        g = self.history.sample(scan.t_end_ns)
                        gt = GroundTruth(scan.t_end_ns, scan.seq, g.p[0], g.q[0], g.R[0])
                        for s in self.scan_sinks:
                            s(scan, gt)
                    scan_idx += 1
                if self.stop_event.is_set():
                    return
            if self.motion.finished:
                return
            # advance one tick
            k += 1
            if cfg.speed > 0.0:
                target = wall0 + k * TICK_NS * 1e-9 / cfg.speed
                delay = target - time.perf_counter()
                if delay > 0.0:
                    self.stop_event.wait(delay)
                elif delay < -0.5 and not warned:
                    warned = True
                    log.warning("simulation is %.2f s behind real time (ray casting too slow?)", -delay)
            self._append(self.motion.step(k * TICK_NS * 1e-9, TICK_NS * 1e-9))

    def start_thread(self) -> threading.Thread:
        t = threading.Thread(target=self.run, name="msim-engine", daemon=True)
        t.start()
        return t

    def stop(self) -> None:
        self.stop_event.set()
