import numpy as np

from msim.pattern import (MID360_EL_MAX_DEG, MID360_EL_MIN_DEG, FilePattern, RosettePattern, load_pattern)


def test_point_count_unit_vectors_and_fov():
    d = RosettePattern().directions(0, 9600)
    assert d.shape == (9600, 3)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    el = np.degrees(np.arcsin(d[:, 2]))
    assert el.min() >= MID360_EL_MIN_DEG - 1e-6 and el.max() <= MID360_EL_MAX_DEG + 1e-6
    assert el.max() > 50 and el.min() < -6                      # the whole vertical FOV is used
    az = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    assert az.min() < -170 and az.max() > 170                    # 360 deg azimuth


def test_firing_order_is_time_order():
    """Point i is fired at i / pps: azimuth advances with time at the configured rate."""
    p = RosettePattern(points_per_second=200_000, az_rev_per_s=60.0)
    d = p.directions(0, 9600)
    az = np.arctan2(d[:, 1], d[:, 0])
    i = 123
    expected = 2 * np.pi * ((60.0 * i / 200_000) % 1.0)
    assert np.isclose(np.cos(az[i]), np.cos(expected)) and np.isclose(np.sin(az[i]), np.sin(expected))
    # consecutive points are close in azimuth (a sweep, not a shuffle)
    step = np.abs(np.angle(np.exp(1j * np.diff(az))))
    assert step.max() < np.radians(1.0)


def test_successive_scans_differ_and_are_deterministic():
    p = RosettePattern()
    a, b, a2 = p.directions(0, 9600), p.directions(1, 9600), p.directions(0, 9600)
    assert not np.allclose(a, b)
    assert np.array_equal(a, a2)


def test_coverage_per_scan():
    d = RosettePattern().directions(3, 9600)
    az, el = np.degrees(np.arctan2(d[:, 1], d[:, 0])), np.degrees(np.arcsin(d[:, 2]))
    cells = set(zip((az // 6).astype(int), (el // 4).astype(int)))
    assert len(cells) / (60 * 15) > 0.85


def test_file_pattern_roundtrip(tmp_path):
    rng = np.random.default_rng(0)
    table = np.stack([rng.uniform(-180, 180, 4 * 9600), rng.uniform(-7, 52, 4 * 9600)], 1)
    f = tmp_path / "pat.npy"
    np.save(f, table)
    p = load_pattern(str(f))
    assert isinstance(p, FilePattern)
    d0, d1, d4 = p.directions(0, 9600), p.directions(1, 9600), p.directions(4, 9600)
    assert d0.shape == (9600, 3) and not np.allclose(d0, d1)
    assert np.allclose(d0, d4)                                   # the table wraps after 4 scans
