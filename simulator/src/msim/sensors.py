"""LiDAR and IMU models driven by the pose history."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import GRAVITY
from .history import PoseHistory
from .pattern import (MID360_POINTS_PER_SCAN, MID360_POINTS_PER_SECOND, ScanPattern)
from .raycast import Caster
from .reflectivity import ReflectivityModel


@dataclass
class ScanData:
    index: int            # position in the simulated timeline (pattern index), not the stream sequence
    t0_ns: int            # timestamp of the first point (= header.timestamp)
    t_end_ns: int         # timestamp of the last point
    xyz: np.ndarray       # (N,3) float32, sensor frame at each point's own time; misses are (0,0,0)
    intensity: np.ndarray  # (N,) uint32
    hit: np.ndarray       # (N,) bool
    seq: int = 0


@dataclass
class LidarModel:
    caster: Caster
    pattern: ScanPattern
    points_per_scan: int = MID360_POINTS_PER_SCAN
    points_per_second: int = MID360_POINTS_PER_SECOND
    min_range: float = 0.1
    max_range: float = 100.0       # m, hard cap; the reflectivity model decides detection (40 m at 10 %)
    range_noise_std: float = 0.0   # m, Gaussian
    # Surface reflectivity -> intensity and detection (see reflectivity.py). None = every surface 40 %.
    reflectivity: ReflectivityModel | None = None
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))

    def point_times_ns(self, t0_ns: int) -> np.ndarray:
        i = np.arange(self.points_per_scan, dtype=np.int64)
        return t0_ns + np.rint(i * (1e9 / self.points_per_second)).astype(np.int64)

    def scan(self, history: PoseHistory, t0_ns: int, index: int) -> ScanData:
        times = self.point_times_ns(t0_ns)
        pose = history.sample(times)  # every point is fired from the pose at its own time
        d_sensor = self.pattern.directions(index, self.points_per_scan)
        d_world = np.einsum("nij,nj->ni", pose.R, d_sensor)
        r, cos, surface = self.caster.cast(pose.p, d_world, self.min_range, self.max_range)
        model = self.reflectivity
        if model is None:
            model = self.reflectivity = ReflectivityModel.uniform(self.caster.n_surfaces)
        hit, inten = model.apply(r, cos, surface, self.rng)
        if self.range_noise_std > 0.0:
            r = np.where(hit, r + self.rng.normal(0.0, self.range_noise_std, r.shape), r)
        xyz = (np.where(hit, r, 0.0)[:, None] * d_sensor).astype(np.float32)   # misses -> (0,0,0), as the real driver
        return ScanData(index, int(times[0]), int(times[-1]), xyz, inten, hit)


@dataclass
class ImuSample:
    t_ns: int
    seq: int
    accel_g: np.ndarray   # (3,) specific force in the sensor frame, in g
    gyro: np.ndarray      # (3,) rad/s


@dataclass
class ImuModel:
    """Ideal IMU on the trajectory, plus optional noise (all defaults zero = clean)."""

    rate_hz: float = 200.0
    accel_noise_density: float = 0.0   # m/s^2/sqrt(Hz)
    gyro_noise_density: float = 0.0    # rad/s/sqrt(Hz)
    accel_bias_walk: float = 0.0       # m/s^2/sqrt(s)  (random walk of the bias)
    gyro_bias_walk: float = 0.0        # rad/s/sqrt(s)
    accel_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)   # constant initial bias, m/s^2
    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)    # constant initial bias, rad/s
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(1))

    def __post_init__(self) -> None:
        self._ba = np.array(self.accel_bias, float)
        self._bg = np.array(self.gyro_bias, float)

    def sample(self, history: PoseHistory, t_ns: int, seq: int) -> ImuSample:
        s = history.sample(t_ns)
        # specific force = R^T (a - g), g = (0, 0, -9.80665); a static level sensor reads +1 g on z
        f = s.R[0].T @ (s.a[0] + np.array([0.0, 0.0, GRAVITY]))
        w = s.w[0].copy()
        dt = 1.0 / self.rate_hz
        if self.accel_bias_walk or self.gyro_bias_walk:
            self._ba = self._ba + self.rng.normal(0, self.accel_bias_walk * np.sqrt(dt), 3)
            self._bg = self._bg + self.rng.normal(0, self.gyro_bias_walk * np.sqrt(dt), 3)
        f = f + self._ba
        w = w + self._bg
        if self.accel_noise_density:
            f = f + self.rng.normal(0, self.accel_noise_density * np.sqrt(self.rate_hz), 3)
        if self.gyro_noise_density:
            w = w + self.rng.normal(0, self.gyro_noise_density * np.sqrt(self.rate_hz), 3)
        return ImuSample(t_ns, seq, f / GRAVITY, w)
