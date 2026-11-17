"""Mid360 scan pattern: ray directions in firing order.

The real Mid360 uses a proprietary non-repetitive pattern; no table is
available, so `RosettePattern` is a documented approximation, and
`FilePattern` loads a real table when one exists (`--pattern file.npy`).

`RosettePattern`: `lasers` beams fired round-robin. Point g (global index, so
successive scans differ) is at time t = g / points_per_second. Azimuth sweeps
360 deg at `az_rev_per_s` revolutions per second; each laser's elevation
oscillates between `el_min` and `el_max` at `el_hz`, the lasers offset in phase
by 1/lasers of a period (defaults: 6 lasers, 60 rev/s, 53.3 Hz; they cover about 92 % of
6 x 4 degree azimuth/elevation cells in one 9600-point scan, measured). With incommensurate frequencies the pattern does not
repeat, and scan k+1 fills the gaps of scan k. Points are returned in firing
order, which is what mslam's per-point timing (`i / points_per_second`) needs.

Sensor frame: x forward, y left, z up; azimuth is measured from +x towards +y.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np

MID360_POINTS_PER_SCAN = 9600
MID360_POINTS_PER_SECOND = 200_000
MID360_SCAN_PERIOD_NS = 48_000_000
MID360_EL_MIN_DEG = -7.0
MID360_EL_MAX_DEG = 52.0


class ScanPattern(Protocol):
    def directions(self, scan_index: int, n: int) -> np.ndarray:
        """(n, 3) unit vectors in the sensor frame, in firing order, for scan `scan_index`."""
        ...


def dirs_from_az_el(az: np.ndarray, el: np.ndarray) -> np.ndarray:
    ce = np.cos(el)
    return np.stack([ce * np.cos(az), ce * np.sin(az), np.sin(el)], axis=-1)


class RosettePattern:
    def __init__(self, points_per_second: int = MID360_POINTS_PER_SECOND, lasers: int = 6,
                 az_rev_per_s: float = 60.0, el_hz: float = 53.3,
                 el_min_deg: float = MID360_EL_MIN_DEG, el_max_deg: float = MID360_EL_MAX_DEG) -> None:
        if lasers < 1 or points_per_second <= 0 or not el_min_deg < el_max_deg:
            raise ValueError("invalid rosette parameters")
        self.pps = points_per_second
        self.lasers = lasers
        self.az_rev = az_rev_per_s
        self.el_hz = el_hz
        self.el_c = np.radians((el_min_deg + el_max_deg) / 2.0)
        self.el_a = np.radians((el_max_deg - el_min_deg) / 2.0)

    def directions(self, scan_index: int, n: int) -> np.ndarray:
        g = scan_index * n + np.arange(n, dtype=np.int64)
        t = g / self.pps  # g < 2^53 so exact enough; the phase below stays well conditioned
        az = 2.0 * np.pi * np.mod(self.az_rev * t, 1.0)
        laser = g % self.lasers
        phase = np.mod(self.el_hz * t + laser / self.lasers, 1.0)
        el = self.el_c + self.el_a * np.sin(2.0 * np.pi * phase)
        return dirs_from_az_el(az, el)


class FilePattern:
    """Pattern from a .npy table: (M, 3) unit vectors or (M, 2) [azimuth_deg, elevation_deg].

    Scan k uses rows [k*n, (k+1)*n) modulo M, so a table of M = n rows repeats every scan
    and one of M = 100*n rows cycles through 100 different scans.
    """

    def __init__(self, path: str | Path) -> None:
        table = np.load(path)
        if table.ndim != 2 or table.shape[1] not in (2, 3):
            raise ValueError(f"{path}: expected shape (M, 2) or (M, 3), got {table.shape}")
        if table.shape[1] == 2:
            table = dirs_from_az_el(np.radians(table[:, 0]), np.radians(table[:, 1]))
        norms = np.linalg.norm(table, axis=1, keepdims=True)
        if not np.all(norms > 0):
            raise ValueError(f"{path}: zero-length direction")
        self.table = (table / norms).astype(np.float64)

    def directions(self, scan_index: int, n: int) -> np.ndarray:
        m = self.table.shape[0]
        idx = (scan_index * n + np.arange(n, dtype=np.int64)) % m
        return self.table[idx]


def load_pattern(spec: str | None, points_per_second: int = MID360_POINTS_PER_SECOND) -> ScanPattern:
    if spec is None or spec == "rosette":
        return RosettePattern(points_per_second)
    return FilePattern(spec)
