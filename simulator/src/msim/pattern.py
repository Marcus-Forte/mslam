"""Mid360 scan pattern: ray directions in firing order.

How the real Mid360 scans (measured on `test/data/living_room_garden.pbscan`, 2026-11-17):

* 4 beams fire in turn (point i belongs to beam i % 4); within a group of 4 points the beams sit about
  0.8 deg apart in azimuth and 0.3 deg apart in elevation.
* Azimuth spins fast: 1.30 deg per 4-point group, about 181 revolutions per second, 8.7 turns per scan.
* Elevation nods slowly between about -7 and 52 deg with a period of about 102 ms (2.1 scans, alternating
  100 / 105 ms between peaks), not locked to the 48 ms scans. So one scan sees a band of elevation, which
  changes from scan to scan: a scan that falls on a bottom turn covers only -7 to 18 deg, the next one the
  whole range. The time spent near the turns puts most points near -5..0 and 40..50 deg.

`RecordedPattern` (the default when its table exists) replays the directions measured in a recording:
`msim gen-pattern` reads a `.pbscan`, takes the direction of every return in the sensor frame (the
recording's motion does not matter, the directions are the sensor's own), and fills the misses (zero
points) by interpolating each beam over time: azimuth as the 1.30 deg/group spin plus an interpolated
residual, elevation linearly. Scan k of the simulator uses scan k of the recording, cycling.

`RosettePattern` is the old synthetic approximation (fast azimuth, sinusoidal elevation per laser): it covers
the whole field of view in every scan and revisits the same directions, unlike the real sensor. It remains
for tests and as the fallback when no table is available. `FilePattern` loads any (M,3) / (M,2) table.

Points are returned in firing order, which is what mslam's per-point timing (`i / points_per_second`)
needs. Sensor frame: x forward, y left, z up; azimuth is measured from +x towards +y.
"""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[2]          # simulator/
DEFAULT_PATTERN_FILE = PACKAGE_ROOT / "out" / "mid360_pattern.npz"
DEFAULT_RECORDING = PACKAGE_ROOT.parent / "test" / "data" / "living_room_garden.pbscan"
MID360_BEAMS = 4

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


def _wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def directions_from_points(xyz: np.ndarray, beams: int = MID360_BEAMS, min_range: float = 0.1
                           ) -> tuple[np.ndarray, np.ndarray, dict]:
    """Azimuth / elevation (deg, float64) of every point of a contiguous point stream, misses filled.

    `xyz` is (N, 3) in firing order across consecutive scans (scans of a Mid360 recording are contiguous:
    9600 points 5 us apart, 48 ms per scan). Points closer than `min_range` (the driver's (0,0,0) misses)
    have no direction; each one is interpolated from the nearest returns of the same beam: the azimuth as a
    constant spin rate plus a slowly varying residual (unwrapped), the elevation linearly.
    """
    xyz = np.asarray(xyz, np.float64)
    n = len(xyz)
    if n % beams:
        raise ValueError(f"point count {n} is not a multiple of {beams} beams")
    r = np.linalg.norm(xyz, axis=1)
    valid = r > min_range
    az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    el = np.degrees(np.arcsin(np.clip(xyz[:, 2] / np.maximum(r, 1e-12), -1.0, 1.0)))
    out_az, out_el = np.empty(n), np.empty(n)
    rates = []
    for b in range(beams):
        idx = np.arange(b, n, beams)
        g = idx // beams                                    # group (firing) index
        vb = valid[idx]
        if vb.sum() < 2:
            raise ValueError(f"beam {b} has fewer than 2 returns")
        gv, a, e = g[vb], az[idx][vb], el[idx][vb]
        step = np.diff(gv) == 1
        if not step.any():
            raise ValueError(f"beam {b}: no two consecutive returns to measure the spin rate")
        rate = float(np.median(_wrap_deg(np.diff(a))[step]))
        res = np.degrees(np.unwrap(np.radians(_wrap_deg(a - rate * gv))))
        out_az[idx] = _wrap_deg(np.interp(g, gv, res) + rate * g)
        out_el[idx] = np.interp(g, gv, e)
        out_az[idx[vb]], out_el[idx[vb]] = a, e            # returns keep their measured direction exactly
        rates.append(rate)
    info = {"filled_fraction": float(1.0 - valid.mean()), "spin_deg_per_group": rates}
    return out_az, out_el, info


def read_recording_points(path: str | Path, max_scans: int | None = None,
                          points_per_scan: int = MID360_POINTS_PER_SCAN) -> np.ndarray:
    """All scan points of a msensor `.pbscan` recording, concatenated, (N, 3) float64."""
    from .recorder import iter_pbscan

    chunks = []
    for entry in iter_pbscan(path):
        if not entry.HasField("scan"):
            continue
        s = entry.scan
        if len(s.x) != points_per_scan:
            raise ValueError(f"{path}: a scan has {len(s.x)} points, expected {points_per_scan}")
        chunks.append(np.stack([np.array(s.x), np.array(s.y), np.array(s.z)], axis=1))
        if max_scans is not None and len(chunks) >= max_scans:
            break
    if not chunks:
        raise ValueError(f"{path}: no scans")
    return np.concatenate(chunks)


def build_recorded_pattern(recording: str | Path, out: str | Path, max_scans: int | None = None,
                           points_per_scan: int = MID360_POINTS_PER_SCAN) -> dict:
    """`msim gen-pattern`: measure the scan pattern of a recording and save it as an .npz table."""
    pts = read_recording_points(recording, max_scans, points_per_scan)
    az, el, info = directions_from_points(pts)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, az_deg=az.astype(np.float32), el_deg=el.astype(np.float32),
             points_per_scan=np.int64(points_per_scan), source=str(Path(recording).name))
    info.update(scans=len(pts) // points_per_scan, path=str(out))
    return info


class RecordedPattern:
    """Directions measured in a real recording (`msim gen-pattern`); scan k replays recorded scan k, cycling."""

    def __init__(self, path: str | Path) -> None:
        with np.load(path) as d:
            self.az = np.radians(d["az_deg"].astype(np.float64))
            self.el = np.radians(d["el_deg"].astype(np.float64))
            self.points_per_scan = int(d["points_per_scan"])
            self.source = str(d["source"])
        if self.az.shape != self.el.shape or len(self.az) % self.points_per_scan:
            raise ValueError(f"{path}: malformed pattern table")
        self.n_scans = len(self.az) // self.points_per_scan

    def directions(self, scan_index: int, n: int) -> np.ndarray:
        if n != self.points_per_scan:
            raise ValueError(f"recorded pattern has {self.points_per_scan} points per scan, asked for {n}")
        k = scan_index % self.n_scans
        sl = slice(k * n, (k + 1) * n)
        return dirs_from_az_el(self.az[sl], self.el[sl])


PATTERN_CHOICES = ("measured", "rosette")


def load_pattern(spec: str | None, points_per_second: int = MID360_POINTS_PER_SECOND) -> ScanPattern:
    """The `--pattern` option:

    * `measured` (default, also None): the pattern measured on the real sensor, i.e. the table at
      `out/mid360_pattern.npz`, built from `test/data/living_room_garden.pbscan` the first time (a few
      seconds). Raises if neither exists (the recording is in Git LFS); there is no silent fallback.
    * `rosette`: the synthetic approximation.
    * a file path: an .npz written by `msim gen-pattern` (e.g. from another recording), or an .npy table.
    """
    import logging

    log = logging.getLogger("msim.pattern")
    if spec == "rosette":
        return RosettePattern(points_per_second)
    if spec is None or spec == "measured":
        if not DEFAULT_PATTERN_FILE.exists():
            if not DEFAULT_RECORDING.exists() or DEFAULT_RECORDING.stat().st_size < 1_000_000:
                raise FileNotFoundError(
                    f"--pattern measured needs {DEFAULT_PATTERN_FILE} or the recording to build it from, "
                    f"{DEFAULT_RECORDING} (run `git lfs pull` in mslam). Alternatives: `msim gen-pattern "
                    f"<recording.pbscan>`, a pattern file, or `--pattern rosette` (synthetic).")
            log.info("building the measured Mid360 pattern from %s (once) -> %s", DEFAULT_RECORDING,
                     DEFAULT_PATTERN_FILE)
            build_recorded_pattern(DEFAULT_RECORDING, DEFAULT_PATTERN_FILE)
        return RecordedPattern(DEFAULT_PATTERN_FILE)
    path = Path(spec)
    if not path.is_file():
        raise ValueError(f"--pattern {spec!r}: expected one of {', '.join(PATTERN_CHOICES)} or an existing file")
    if path.suffix == ".npz":
        return RecordedPattern(path)
    return FilePattern(path)
