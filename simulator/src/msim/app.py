"""Wiring: scene + motion + sensors + engine + gRPC server + optional writers."""

from __future__ import annotations

import logging
import signal
import threading
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .engine import EngineConfig, SimEngine
from .motion import GameController, Limits, MotionSource, load_trajectory
from .pattern import MID360_POINTS_PER_SCAN, MID360_POINTS_PER_SECOND, load_pattern
from .raycast import BoxCaster, MujocoCaster
from .recorder import GroundTruthWriter, PbscanWriter
from .reflectivity import ReflectivityModel
from .scene import Scene, load_scene
from .sensors import ImuModel, LidarModel
from .streams import SensorHub, SensorServer

log = logging.getLogger("msim")


@dataclass
class SimOptions:
    scene: str
    port: int = 50051
    host: str = "0.0.0.0"
    trajectory: str | None = None
    headless: bool = False
    speed: float = 1.0                 # 1 = real time, 0 = unpaced
    seed: int = 0
    gt_log: str | None = None
    record: str | None = None
    caster: str = "box"                # box | mujoco
    pattern: str | None = None
    range_noise: float = 0.0
    intensity_noise: float = 0.0       # relative std of the reported reflectivity
    imu_accel_noise: float = 0.0
    imu_gyro_noise: float = 0.0
    imu_accel_bias_walk: float = 0.0
    imu_gyro_bias_walk: float = 0.0
    imu_accel_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wait_for_clients: bool | None = None   # default: on when headless
    exit_when_done: bool = False
    backpressure: str = "auto"         # auto = block when unpaced, drop when paced
    start_ns: int = 1_000_000_000_000
    show_points: bool = True
    show_ui: bool = False              # viewer: show MuJoCo's left/right panels
    limits: Limits = field(default_factory=Limits)


class Simulation:
    def __init__(self, opts: SimOptions) -> None:
        self.opts = opts
        self.scene: Scene = load_scene(opts.scene)
        self.controller: GameController | None = None
        motion: MotionSource
        if opts.trajectory:
            motion = load_trajectory(opts.trajectory)
        else:
            if opts.headless:
                raise ValueError("--headless needs --trajectory (there is nobody to fly the sensor)")
            self.controller = GameController(self.scene.start_position, self.scene.start_yaw, opts.limits, self.scene)
            motion = self.controller
        self.motion = motion

        caster = BoxCaster(self.scene) if opts.caster == "box" else MujocoCaster(self.scene)
        lidar = LidarModel(caster, load_pattern(opts.pattern, MID360_POINTS_PER_SECOND), MID360_POINTS_PER_SCAN,
                           MID360_POINTS_PER_SECOND, range_noise_std=opts.range_noise,
                           reflectivity=ReflectivityModel.from_scene(self.scene, opts.intensity_noise),
                           rng=np.random.default_rng(opts.seed))
        imu = ImuModel(accel_noise_density=opts.imu_accel_noise, gyro_noise_density=opts.imu_gyro_noise,
                       accel_bias_walk=opts.imu_accel_bias_walk, gyro_bias_walk=opts.imu_gyro_bias_walk,
                       accel_bias=opts.imu_accel_bias, gyro_bias=opts.imu_gyro_bias,
                       rng=np.random.default_rng(opts.seed + 1))
        self.engine = SimEngine(motion, lidar, imu, EngineConfig(start_ns=opts.start_ns, speed=opts.speed))

        block = (opts.speed == 0.0) if opts.backpressure == "auto" else opts.backpressure == "block"
        self.hub = SensorHub(block)
        self.server = SensorServer(self.hub, f"{opts.host}:{opts.port}")
        self.port = self.server.port

        self._writers: list = []
        self.engine.imu_sinks.append(self.hub.on_imu)
        self.engine.scan_sinks.append(self.hub.on_scan)
        always_scans = False
        if opts.gt_log:
            gt = GroundTruthWriter(opts.gt_log)
            self.engine.scan_sinks.append(gt)
            self._writers.append(gt)
            always_scans = True
        if opts.record:
            rec = PbscanWriter(opts.record)
            self.engine.imu_sinks.append(rec.on_imu)
            self.engine.scan_sinks.append(rec.on_scan)
            self._writers.append(rec)
            always_scans = True
        self.points = None
        if not opts.headless and opts.show_points:
            from .viewer import ScanPoints
            self.points = ScanPoints()
            self.engine.scan_sinks.append(self.points)
            always_scans = True
        self.engine.scan_wanted = lambda: always_scans or self.hub.lidar_wanted()

        wait = opts.headless if opts.wait_for_clients is None else opts.wait_for_clients
        if wait:
            self.engine.start_gate = lambda ev: self.hub.wait_for_clients(ev)
        self.engine.on_finish.append(self._finished)
        self.thread: threading.Thread | None = None

    def _finished(self) -> None:
        for w in self._writers:
            w.close()
        self._writers.clear()
        if self.opts.exit_when_done and not self.engine.stop_event.is_set():
            log.info("Simulation finished; closing the streams.")
            self.hub.close()

    def start(self) -> None:
        self.server.start()
        self.thread = self.engine.start_thread()

    def stop(self) -> None:
        self.engine.stop()
        if self.thread:
            self.thread.join(timeout=5.0)
        self.server.stop(1.0)
        for w in self._writers:
            w.close()
        self._writers.clear()

    def run_headless(self) -> None:
        """Serve until the engine finishes (with exit_when_done) or Ctrl-C."""
        stop = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        self.start()
        print(f"msim serving on port {self.port} ({'real time' if self.opts.speed else 'unpaced'}); Ctrl-C to quit")
        while not stop.is_set():
            if self.engine.done.is_set():
                if self.opts.exit_when_done:
                    stop.wait(1.0)  # let the clients drain their queues
                    break
                print("Trajectory finished; still serving (Ctrl-C to quit).")
                stop.wait()
                break
            stop.wait(0.2)
        self.stop()

    def run_viewer(self) -> None:
        from .viewer import KeyboardControl, run_viewer
        self.start()
        print(f"msim serving on port {self.port}")
        kb = KeyboardControl(self.controller) if self.controller else None
        try:
            run_viewer(self.scene, self.engine, kb, self.points, self.opts.show_ui)
        finally:
            self.stop()
