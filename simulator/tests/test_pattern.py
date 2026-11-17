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


# ---------------------------------------------------------------------------
# Pattern measured from a recording
# ---------------------------------------------------------------------------

import pytest  # noqa: E402

from msim import pattern as pattern_mod  # noqa: E402
from msim.pattern import (DEFAULT_RECORDING, RecordedPattern, build_recorded_pattern,  # noqa: E402
                          directions_from_points)


def synthetic_mid360(n_scans=6, n=9600):
    """A Mid360-like stream: 4 beams, 1.30 deg per group azimuth spin, ~102 ms elevation nod."""
    i = np.arange(n_scans * n)
    g, b = i // 4, i % 4
    t = i * 5e-6
    az = (1.30 * g + 0.8 * b + 2.0 * np.sin(np.radians(1.30 * g))) % 360.0 - 180.0
    el = 22.5 + 29.5 * np.sin(2 * np.pi * t / 0.102) - 0.3 * b
    u = np.stack([np.cos(np.radians(el)) * np.cos(np.radians(az)), np.cos(np.radians(el)) * np.sin(np.radians(az)),
                  np.sin(np.radians(el))], 1)
    r = 3.0 + np.sin(i * 0.01)
    return u * r[:, None], az, el


def ang_err_deg(az1, el1, az2, el2):
    u1 = pattern_mod.dirs_from_az_el(np.radians(az1), np.radians(el1))
    u2 = pattern_mod.dirs_from_az_el(np.radians(az2), np.radians(el2))
    return np.degrees(np.arccos(np.clip(np.sum(u1 * u2, 1), -1, 1)))


def test_returns_keep_their_direction_and_misses_are_interpolated_per_beam():
    xyz, az, el = synthetic_mid360()
    rng = np.random.default_rng(0)
    miss = rng.random(len(xyz)) < 0.2
    for s in rng.choice(len(xyz) - 400, 40, replace=False):     # and some 0.2 to 2 ms runs (as in the recording)
        miss[s:s + rng.integers(40, 400)] = True
    xyz[miss] = 0.0
    a, e, info = directions_from_points(xyz)
    assert info["filled_fraction"] == pytest.approx(miss.mean())
    assert all(r == pytest.approx(1.30, abs=0.01) for r in info["spin_deg_per_group"])
    assert np.max(ang_err_deg(a[~miss], e[~miss], az[~miss], el[~miss])) < 1e-5     # arccos rounding
    err = ang_err_deg(a[miss], e[miss], az[miss], el[miss])
    assert np.median(err) < 0.2 and np.percentile(err, 99) < 2.0


def test_recorded_pattern_roundtrip_and_cycle(tmp_path, monkeypatch):
    xyz, _, _ = synthetic_mid360(n_scans=3)
    monkeypatch.setattr(pattern_mod, "read_recording_points", lambda path, max_scans=None, n=9600: xyz)
    info = build_recorded_pattern("fake.pbscan", tmp_path / "p.npz")
    assert info["scans"] == 3
    p = pattern_mod.load_pattern(str(tmp_path / "p.npz"))
    assert isinstance(p, RecordedPattern) and p.n_scans == 3
    d0 = p.directions(0, 9600)
    assert d0.shape == (9600, 3) and np.allclose(np.linalg.norm(d0, axis=1), 1.0)
    assert np.allclose(d0, xyz[:9600] / np.linalg.norm(xyz[:9600], axis=1, keepdims=True), atol=1e-6)
    assert np.array_equal(p.directions(3, 9600), d0) and not np.allclose(p.directions(1, 9600), d0)
    with pytest.raises(ValueError):
        p.directions(0, 4800)


def test_measured_without_table_or_recording_is_an_error_not_a_silent_fallback(tmp_path, monkeypatch):
    monkeypatch.setattr(pattern_mod, "DEFAULT_PATTERN_FILE", tmp_path / "none.npz")
    monkeypatch.setattr(pattern_mod, "DEFAULT_RECORDING", tmp_path / "none.pbscan")
    for spec in ("measured", None):
        with pytest.raises(FileNotFoundError, match="git lfs pull"):
            pattern_mod.load_pattern(spec)
    assert isinstance(pattern_mod.load_pattern("rosette"), RosettePattern)


def test_unknown_pattern_name_is_rejected():
    with pytest.raises(ValueError, match="measured, rosette"):
        pattern_mod.load_pattern("auto")


@pytest.mark.skipif(not DEFAULT_RECORDING.exists() or DEFAULT_RECORDING.stat().st_size < 1_000_000,
                    reason="living_room_garden.pbscan not present (Git LFS)")
def test_pattern_measured_from_the_real_recording_scans_elevation_slices(tmp_path):
    """The real Mid360 nods in elevation every ~102 ms while it spins 181 rev/s, so a 48 ms scan covers a band of
    elevation that changes from scan to scan; the measured table must reproduce that."""
    info = build_recorded_pattern(DEFAULT_RECORDING, tmp_path / "p.npz", max_scans=60)
    assert info["scans"] == 60
    assert 0.05 < info["filled_fraction"] < 0.4
    assert all(r == pytest.approx(1.30, abs=0.03) for r in info["spin_deg_per_group"])
    p = RecordedPattern(tmp_path / "p.npz")
    spans = []
    for k in range(60):
        el = np.degrees(np.arcsin(p.directions(k, 9600)[:, 2]))
        spans.append(np.percentile(el, 98) - np.percentile(el, 2))
        assert el.min() > -10 and el.max() < 56
    assert min(spans) < 35 and max(spans) > 50                    # some scans see a slice, some the whole range
