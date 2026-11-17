"""End to end: real gRPC server, real generated client, scripted headless run."""

import struct
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from msim.app import SimOptions, Simulation
from msim.check import run_check
from msim.compare import compare, gt_in_slam_frame, load_gt, mslam_euler_to_rot, Trajectory
from msim.geometry import rot_z

import recording_pb2  # noqa: E402  (path added by msim)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def short_traj(tmp_path):
    p = tmp_path / "t.yaml"
    p.write_text(yaml.safe_dump({"speed": 1.0, "lead_in": 0.5, "lead_out": 0.5, "waypoints": [
        {"p": [-0.5, -0.5, 1.0], "yaw_deg": 0}, {"p": [0.5, 0.0, 1.0]}, {"p": [-0.5, 0.5, 1.2]}]}))
    return str(p)


def make(traj, tmp_path, tag="a", **kw):
    return Simulation(SimOptions(scene=str(ROOT / "scenes" / "single_room.yaml"), trajectory=traj, headless=True,
                                 port=0, host="127.0.0.1", speed=0.0, exit_when_done=True,
                                 gt_log=str(tmp_path / f"gt_{tag}.csv"), record=str(tmp_path / f"rec_{tag}.pbscan"), **kw))


def read_pbscan(path):
    scans, imus = [], []
    data = Path(path).read_bytes()
    i = 0
    while i < len(data):
        (n,) = struct.unpack_from("<Q", data, i)
        e = recording_pb2.RecordingEntry()
        e.ParseFromString(data[i + 8:i + 8 + n])
        (scans if e.WhichOneof("entry") == "scan" else imus).append(e.scan if e.WhichOneof("entry") == "scan" else e.imu)
        i += 8 + n
    return scans, imus


def test_unpaced_headless_run(short_traj, tmp_path):
    sim = make(short_traj, tmp_path)
    sim.start()
    t0 = time.time()
    rep = run_check(f"127.0.0.1:{sim.port}", seconds=60)
    assert sim.engine.done.wait(10)
    sim.stop()
    assert time.time() - t0 < 30

    total = sim.motion.total_duration
    # Both streams complete, lossless, in order, on the nominal grid, and end cleanly.
    assert rep.imu.error == "" and rep.scan.error == ""
    assert rep.imu.ended_cleanly and rep.scan.ended_cleanly
    assert rep.imu.gaps == 0 and rep.scan.gaps == 0
    assert rep.imu.non_monotonic == 0 and rep.scan.non_monotonic == 0
    assert set(rep.imu.dts) == {5_000_000}
    assert set(rep.scan.dts) == {48_000_000}
    assert rep.imu.count == pytest.approx(total / 0.005, abs=3)
    assert rep.scan.count == pytest.approx((total - 0.048) / 0.048, abs=3)
    assert set(rep.points) == {9600}
    assert 0 < rep.min_hit_intensity <= 150               # diffuse surfaces only (reflectivity.py)
    assert rep.miss_fraction == 0.0                      # closed room: every ray hits
    assert rep.last_imu_az_g == pytest.approx(1.0, abs=0.2)

    # Ground truth: one row per scan, at the last point's time, on the same clock.
    gt = load_gt(str(tmp_path / "gt_a.csv"))
    assert len(gt.t_ns) == rep.scan.count
    scans, imus = read_pbscan(tmp_path / "rec_a.pbscan")
    assert len(scans) == rep.scan.count and len(imus) == rep.imu.count
    assert np.array_equal(gt.t_ns, [s.header.timestamp + 9599 * 5000 for s in scans])
    assert [s.header.sequence_number for s in scans] == list(range(len(scans)))
    # an IMU sample lies inside the scan window (both streams share one clock)
    assert imus[0].header.timestamp <= scans[0].header.timestamp + 5_000_000
    # no threads left behind
    assert not [t for t in threading.enumerate() if t.name == "msim-engine"]


def test_reproducible_with_a_fixed_seed(short_traj, tmp_path):
    outs = []
    for tag in ("a", "b"):
        sim = make(short_traj, tmp_path, tag, seed=7, range_noise=0.01, imu_accel_noise=0.02)
        sim.start()
        run_check(f"127.0.0.1:{sim.port}", seconds=60)
        sim.engine.done.wait(10)
        sim.stop()
        outs.append(((tmp_path / f"gt_{tag}.csv").read_bytes(), (tmp_path / f"rec_{tag}.pbscan").read_bytes()))
    assert outs[0][0] == outs[1][0]
    assert outs[0][1] == outs[1][1]


def test_paced_run_follows_wall_time(short_traj, tmp_path):
    sim = Simulation(SimOptions(scene=str(ROOT / "scenes" / "single_room.yaml"), trajectory=short_traj, headless=True,
                                port=0, host="127.0.0.1", speed=1.0, wait_for_clients=True))
    sim.start()
    rep = run_check(f"127.0.0.1:{sim.port}", seconds=2.0)
    sim.stop()
    assert 180 <= rep.imu.count / rep.wall_s <= 210
    assert 19 <= rep.scan.count / rep.wall_s <= 23
    assert rep.imu.gaps == 0 and rep.scan.gaps == 0      # a prompt client loses nothing in drop mode either


def test_headless_requires_trajectory(tmp_path):
    with pytest.raises(ValueError):
        Simulation(SimOptions(scene=str(ROOT / "scenes" / "single_room.yaml"), headless=True, port=0))


def test_pbscan_matches_msensor_framing(short_traj, tmp_path):
    """uint64 LE length + RecordingEntry, exactly what ScanRecorder::append writes."""
    sim = make(short_traj, tmp_path)
    sim.start()
    run_check(f"127.0.0.1:{sim.port}", seconds=60)
    sim.engine.done.wait(10)
    sim.stop()
    raw = (tmp_path / "rec_a.pbscan").read_bytes()
    (n,) = struct.unpack_from("<Q", raw, 0)
    assert 8 + n < len(raw)
    first = recording_pb2.RecordingEntry()
    first.ParseFromString(raw[8:8 + n])
    assert first.WhichOneof("entry") == "imu"


# ---------------- compare -------------------------------------------------

def test_gt_in_slam_frame_removes_initial_pose():
    yaw0 = 0.7
    R0 = rot_z(yaw0)
    gt = Trajectory(np.array([1, 2]), np.array([[5.0, 3.0, 1.0], [6.0, 3.0, 1.0]]), np.stack([R0, R0 @ rot_z(0.1)]))
    ref = gt_in_slam_frame(gt)
    assert np.allclose(ref.p[0], 0) and np.allclose(ref.R[0], np.eye(3))
    # moved 1 m along the initial heading's world x: in the start frame that is (cos, -sin)
    assert np.allclose(ref.p[1], [np.cos(yaw0), -np.sin(yaw0), 0])


def test_compare_zero_error_when_identical():
    t = np.arange(5) * 48_000_000 + 47_995_000
    gt = Trajectory(t, np.cumsum(np.ones((5, 3)) * 0.1, 0), np.tile(np.eye(3), (5, 1, 1)))
    est = Trajectory(t - 47_995_000, gt_in_slam_frame(gt).p, gt_in_slam_frame(gt).R)
    err = compare(gt, est)
    assert err.n == 5 and err.pos_err.max() < 1e-12


def test_mslam_euler_convention():
    # R = Rx Ry Rz: pure yaw is a rotation about z
    assert np.allclose(mslam_euler_to_rot(0, 0, 0.3), rot_z(0.3))


def test_serve_arguments_are_listed_with_defaults_marked():
    import argparse

    from msim.cli import _add_serve, format_arguments

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd")
    _add_serve(sub)
    a = ap.parse_args(["serve", "--scene", "s.yaml", "--port", "6000", "--pattern", "rosette"])
    text = format_arguments(a, a._parser)
    lines = {ln.split()[0]: ln for ln in text.splitlines()}
    assert "verbose" not in lines and "cmd" not in lines and "_parser" not in lines
    assert lines["scene"].split()[1] == "s.yaml" and "(default)" not in lines["scene"]
    assert lines["port"].split()[1] == "6000" and "(default)" not in lines["port"]
    assert "rosette" in lines["pattern"] and "(default)" not in lines["pattern"]
    assert lines["host"].endswith("0.0.0.0 (default)")
    assert lines["gt-log"].endswith("- (default)")
