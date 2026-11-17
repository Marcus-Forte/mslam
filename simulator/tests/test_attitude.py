"""Full 3D attitude: roll, unlimited pitch, level/body translation, keys, UI options."""

import hashlib
import math
import struct
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import pytest

from msim import viewer
from msim.engine import kin_to_history_row
from msim.geometry import GRAVITY, exp_so3, rot_from_yaw_pitch, so3_log
from msim.history import PoseHistory
from msim.motion import Command, GameController, Waypoint, ScriptedTrajectory, kin_from_euler, load_trajectory
from msim.raycast import BoxCaster
from msim.sensors import ImuModel, LidarModel

from test_sensors import FixedPattern, const_kin, fill_history

ROOT = Path(__file__).resolve().parents[1]
DT = 1e-3


def run(ctrl, seconds, command):
    ctrl.command = command
    return [ctrl.step(k * DT, DT) for k in range(1, int(seconds / DT) + 1)]


# ---- static specific force -------------------------------------------------

@pytest.mark.parametrize("roll_deg,pitch_deg", [(30, 0), (0, 30), (30, 30), (180, 0), (150, -40)])
def test_static_attitudes_read_rotated_gravity(roll_deg, pitch_deg):
    roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)
    h = fill_history(lambda t: const_kin([0, 0, 1.0], pitch=pitch, roll=roll), 0.05)
    s = ImuModel().sample(h, h.start_ns + 10_000_000, 0)
    R = rot_from_yaw_pitch(0.0, pitch, roll)
    assert np.allclose(s.accel_g, R.T @ [0, 0, 1.0], atol=1e-9)
    assert np.linalg.norm(s.accel_g) == pytest.approx(1.0)
    if roll_deg == 180 and pitch_deg == 0:
        assert np.allclose(s.accel_g, [0, 0, -1.0], atol=1e-9)    # inverted: reads -1 g on z
    assert np.allclose(s.gyro, 0)


def test_euler_rates_map_to_gyro_axes():
    for kw, expected in [(dict(roll_rate=0.7), [0.7, 0, 0]), (dict(pitch_rate=0.4), [0, -0.4, 0]),
                         (dict(yaw_rate=0.9), [0, 0, 0.9])]:
        k = const_kin([0, 0, 1.0], **kw)
        assert np.allclose(k.w, expected)


def test_euler_w_matches_finite_difference_with_roll():
    yaw, yr, pit, pr, rol, rr = 0.4, 0.7, 0.3, -0.2, -0.8, 0.5
    dt = 1e-6
    k0 = kin_from_euler(np.zeros(3), np.zeros(3), np.zeros(3), yaw, yr, pit, pr, rol, rr)
    R1 = rot_from_yaw_pitch(yaw + yr * dt, pit + pr * dt, rol + rr * dt)
    assert np.allclose(so3_log(k0.R.T @ R1) / dt, k0.w, atol=1e-5)


# ---- controller: attitude --------------------------------------------------

def integrate_gyro_error(history: PoseHistory):
    """Max angle between the history attitude and R <- R Exp(w dt) from the history's own w."""
    R = history.sample(history.start_ns).R[0]
    worst = 0.0
    for k in range(1, history.count):
        t = history.start_ns + k * 1_000_000
        s = history.sample(t)
        R = R @ exp_so3(s.w[0] * DT)
        worst = max(worst, np.linalg.norm(so3_log(s.R[0].T @ R)))
    return worst


def test_gyro_integrates_back_to_attitude_after_a_long_mixed_manoeuvre():
    c = GameController([0, 0, 1.0], 0.3)
    h = PoseHistory(10**12, 16384)
    h.append(*kin_to_history_row(c.initial()))
    plan = [(2.0, Command(roll_rate=1.0)), (2.0, Command(pitch_rate=1.0)), (2.0, Command(yaw_rate=1.0, roll_rate=-1.0)),
            (3.0, Command(roll_rate=1.0, pitch_rate=1.0, yaw_rate=1.0)), (2.0, Command(pitch_rate=-1.0, roll_rate=0.5)),
            (1.0, Command())]
    k = 0
    for secs, cmd in plan:
        c.command = cmd
        for _ in range(int(secs / DT)):
            k += 1
            h.append(*kin_to_history_row(c.step(k * DT, DT)))
    assert integrate_gyro_error(h) < 1e-6
    assert np.allclose(c.R.T @ c.R, np.eye(3), atol=1e-12) and np.linalg.det(c.R) == pytest.approx(1.0, abs=1e-12)
    assert np.isfinite(h._q[:h.count]).all()


def test_controller_goes_past_90_degrees_pitch_and_inverted():
    c = GameController([0, 0, 1.0], 0.0)
    ks = run(c, 8.0, Command(pitch_rate=1.0))        # 0.8 rad/s * ~7.5 s = well over pi
    x_up = np.array([k.R[2, 0] for k in ks])          # z component of the body x axis = sin(pitch_up)
    z_up = np.array([k.R[2, 2] for k in ks])
    assert x_up.max() > 0.999999                       # straight up reached ...
    assert z_up.min() < -0.5                           # ... and passed: the sensor is upside down
    assert all(np.isfinite(k.R).all() and np.isfinite(k.w).all() for k in ks)
    # the translation channels keep working in that attitude (level mode keeps the last heading)
    ks2 = run(c, 1.0, Command(forward=1.0))
    assert np.isfinite(ks2[-1].p).all()


def test_level_key_returns_to_z_up_and_keeps_heading():
    yaw0 = 0.8
    c = GameController([0, 0, 1.0], 0.0)
    c.R = rot_from_yaw_pitch(yaw0, math.radians(-50), math.radians(40))   # roll 40, pitch -50 (nose down)
    h0 = math.atan2(c.R[1, 0], c.R[0, 0])
    ks = run(c, 6.0, Command(level=True))
    R = ks[-1].R
    assert R[2, 2] > 0.9999                                    # z axis up again
    h1 = math.atan2(R[1, 0], R[0, 0])
    assert abs(math.remainder(h1 - h0, 2 * math.pi)) < math.radians(10), (h0, h1)
    w = np.array([k.w for k in ks])
    assert np.abs(w).max() < 1.3                                # never faster than the rate limits allow
    assert np.abs(np.diff(w, axis=0)).max() / DT < 40.0         # no jumps: bounded angular acceleration
    assert np.abs(ks[-1].w).max() < 0.05                        # at rest when level


def test_level_from_upside_down():
    c = GameController([0, 0, 1.0], 0.0)
    c.R = rot_from_yaw_pitch(0.0, 0.0, math.pi)                 # exactly inverted
    ks = run(c, 8.0, Command(level=True))
    assert ks[-1].R[2, 2] > 0.999 and np.isfinite(ks[-1].R).all()


def test_reset_restores_attitude_and_clears_channels():
    c = GameController([0.5, 0, 1.0], 0.4)
    run(c, 2.0, Command(roll_rate=1.0, pitch_rate=1.0, forward=1.0))
    c.reset()
    k = c.step(0.0, DT)
    assert np.allclose(k.p, [0.5, 0, 1.0], atol=1e-3) and np.allclose(k.R, rot_from_yaw_pitch(0.4, 0.0), atol=1e-6)
    assert np.abs(k.w).max() < 0.01


# ---- controller: translation frames ---------------------------------------

def test_level_vs_body_translation_modes():
    pitched = rot_from_yaw_pitch(0.0, math.radians(45))
    for mode in ("level", "body"):
        c = GameController([0, 0, 1.0], 0.0, frame_mode=mode)
        c.R = pitched.copy()
        ks = run(c, 1.5, Command(forward=1.0))
        v = ks[-1].v
        assert v[0] > 0.3
        if mode == "level":
            assert abs(v[2]) < 1e-9                             # horizontal
        else:
            assert v[2] == pytest.approx(v[0], rel=1e-6)        # along the tilted x axis (45 deg)
            assert np.allclose(v, v[0] * pitched[:, 0] / pitched[0, 0], atol=1e-9)
    assert c.toggle_frame_mode() == "level" and c.toggle_frame_mode() == "body"


@pytest.mark.parametrize("mode", ["level", "body"])
def test_collision_sliding_in_both_modes(single_room, mode):
    c = GameController([0, 0, 1.0], 0.0, scene=single_room, frame_mode=mode)
    c.R = rot_from_yaw_pitch(0.0, math.radians(30))
    ks = run(c, 8.0, Command(forward=1.0))
    p = np.array([k.p for k in ks])
    assert not any(single_room.collides(x, c.limits.radius - 1e-9) for x in p)
    assert p[:, 0].max() > 1.0                                  # it travelled and reached the obstacles
    assert np.abs(ks[-1].v).max() < 0.05                        # and stopped against them


# ---- LiDAR while rolling ----------------------------------------------------

class SideRays:
    def directions(self, scan_index, n):
        return np.tile([0.0, 1.0, 0.0], (n, 1))                 # sensor +y


def test_per_point_casting_while_rolling(single_room):
    wr = 0.5
    start = 10**12
    h = fill_history(lambda t: const_kin([0, 0, 1.0], roll=wr * t, roll_rate=wr), 0.3, start)
    scan = LidarModel(BoxCaster(single_room), SideRays()).scan(h, start + 100_000_000, 0)
    t = 0.1 + np.arange(9600) / 200_000
    phi = wr * t
    # ray = R(t_i) y = (0, cos phi, sin phi) in the world: it hits the +y wall at 1.9 / cos(phi)
    assert np.allclose(scan.xyz[:, 1], 1.9 / np.cos(phi), atol=3e-5)
    assert np.allclose(scan.xyz[:, [0, 2]], 0.0)


# ---- scripted trajectories ---------------------------------------------------

def test_trajectory_roll_waypoints_and_yaml(tmp_path):
    f = tmp_path / "t.yaml"
    f.write_text("waypoints:\n  - {p: [0,0,1], roll_deg: 0}\n  - {p: [1,0,1], roll_deg: 90}\n  - {p: [2,0,1], roll_deg: 0}\n")
    tr = load_trajectory(f)
    mid = tr.eval(tr.t[1])
    assert mid.R[2, 1] == pytest.approx(1.0, abs=1e-9)           # body y axis points up at roll 90
    assert np.allclose(tr.eval(0.0).w, 0) and np.allclose(tr.eval(tr.duration).w, 0)


def _gt_and_imu_hashes(tmp_path, scans=160):
    from msim.app import SimOptions, Simulation
    import recording_pb2
    sim = Simulation(SimOptions(scene=str(ROOT / "scenes" / "two_rooms.yaml"),
                                trajectory=str(ROOT / "trajectories" / "two_rooms_loop.yaml"), headless=True, port=0,
                                host="127.0.0.1", speed=0.0, wait_for_clients=False,
                                gt_log=str(tmp_path / "gt.csv"), record=str(tmp_path / "r.pbscan")))
    n = [0]

    def stop_after(scan, gt):
        n[0] += 1
        if n[0] >= scans:
            sim.engine.stop()
    sim.engine.scan_sinks.append(stop_after)
    sim.start()
    sim.engine.done.wait(60)
    sim.stop()
    gt_lines = (tmp_path / "gt.csv").read_text().splitlines()[:61]
    data = (tmp_path / "r.pbscan").read_bytes()
    i, c, hh = 0, 0, hashlib.sha256()
    while i < len(data) and c < 1500:
        (k,) = struct.unpack_from("<Q", data, i)
        e = recording_pb2.RecordingEntry()
        e.ParseFromString(data[i + 8:i + 8 + k])
        i += 8 + k
        if e.WhichOneof("entry") == "imu":
            m = e.imu
            hh.update(repr((m.header.timestamp, m.header.sequence_number, m.ax, m.ay, m.az, m.gx, m.gy, m.gz)).encode())
            c += 1
    return hashlib.sha256("\n".join(gt_lines).encode()).hexdigest(), hh.hexdigest()


def test_trajectory_without_roll_is_bit_identical_to_the_pre_roll_output(tmp_path):
    """Hashes of the first 60 ground-truth rows and 1500 IMU samples of two_rooms_loop, captured before
    roll was added (same machine and numpy; a different libm could in principle change the last bit)."""
    gt, imu = _gt_and_imu_hashes(tmp_path)
    assert gt == "fd3b4afd5109f2008298cbff8f09cac35561598390213832931a00e6e6f72b42"
    assert imu == "6f73a5fc9cedc5312e7dd217fb8d022425414e71426024d701c9169f17aa8814"


# ---- keyboard and viewer options ---------------------------------------------

def _keys():
    return viewer


def test_key_mapping():
    c = GameController([0, 0, 1.0], 0.0)
    kb = viewer.KeyboardControl(c)
    expect = {viewer.K_UP: ("forward", 1), viewer.K_DOWN: ("forward", -1), viewer.K_Z: ("left", 1),
              viewer.K_X: ("left", -1), viewer.K_E: ("up", 1), viewer.K_Q: ("up", -1),
              viewer.K_LEFT: ("yaw_rate", 1), viewer.K_RIGHT: ("yaw_rate", -1), viewer.K_Y: ("pitch_rate", 1),
              viewer.K_H: ("pitch_rate", -1), viewer.K_O: ("roll_rate", 1), viewer.K_U: ("roll_rate", -1)}
    for key, (field, sign) in expect.items():
        kb._until.clear()
        kb.on_key(key)
        cmd = c.command
        assert getattr(cmd, field) == sign, field
        assert sum(abs(float(x)) for x in cmd[:6]) == 1.0 and not cmd.level
    kb._until.clear()
    kb.on_key(viewer.K_N)
    assert c.command.level
    mode = c.frame_mode
    kb.on_key(viewer.K_B)
    assert c.frame_mode != mode
    kb.on_key(viewer.K_SPACE)
    assert c.command == Command()


def test_keys_avoid_viewer_hotkeys_that_cannot_be_undone():
    rnd = {s[2] for s in mujoco.mjRNDSTRING if s[2]}
    used = viewer.CONTROL_KEYS | {viewer.K_B, viewer.K_ENTER, viewer.K_SPACE}
    letters = {chr(k) for k in used if 32 < k < 127}
    assert not letters & rnd, letters & rnd
    assert not used & viewer.RESERVED_KEYS and viewer.K_TAB in viewer.RESERVED_KEYS
    vis = {s[2] for s in mujoco.mjVISSTRING if s[2]}
    assert {chr(viewer.K_U), chr(viewer.K_O), chr(viewer.K_B), chr(viewer.K_N)} <= vis   # restored every frame


def test_viewer_ui_option_plumbing(monkeypatch, single_room):
    assert viewer.launch_kwargs(False) == {"show_left_ui": False, "show_right_ui": False}
    assert viewer.launch_kwargs(True) == {"show_left_ui": True, "show_right_ui": True}
    seen = {}

    def fake_launch(model, data, **kw):
        seen.update(kw)
        raise RuntimeError("stop")
    monkeypatch.setattr(mujoco.viewer, "launch_passive", fake_launch)
    for show in (False, True):
        with pytest.raises(RuntimeError):
            viewer.run_viewer(single_room, None, None, None, show_ui=show)
        assert seen["show_left_ui"] is show and seen["show_right_ui"] is show

    # the CLI flag reaches SimOptions (default hidden)
    from msim import app, cli
    got = []

    class FakeSim:
        def __init__(self, opts):
            got.append(opts)

        def run_viewer(self):
            pass
    monkeypatch.setattr(app, "Simulation", FakeSim)
    cli.main(["serve", "--scene", "x.yaml"])
    cli.main(["serve", "--scene", "x.yaml", "--show-ui"])
    assert [o.show_ui for o in got] == [False, True]


def test_level_mode_pitching_over_the_top_keeps_the_velocity_continuous():
    """Pitching through vertical while flying forward must not flip the level-mode velocity in one tick.

    The horizontal projection of x reverses there; taken raw it gave a 4000 m/s^2 spike in the accelerometer.
    """
    from msim.motion import Command, GameController

    c = GameController([0, 0, 1.0], 0.0)
    dt = 1e-3
    a_max = 0.0
    for k in range(1, 8001):
        c.command = Command(forward=1.0, pitch_rate=1.0 if k > 2000 else 0.0)
        kin = c.step(k * dt, dt)
        a_max = max(a_max, float(np.linalg.norm(kin.a)))
    assert c.R[2, 0] < 0.0 or c.R[2, 2] < 0.0      # it did go over the top
    assert a_max < 10.0
    assert kin.v[0] > 1.0                           # still flying the original heading (+x)
