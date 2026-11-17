from pathlib import Path

import numpy as np
import pytest

from msim.motion import GameController, Limits, ScriptedTrajectory, Waypoint, load_trajectory, rect_loop

ROOT = Path(__file__).resolve().parents[1]
DT = 1e-3


def run(ctrl, seconds, command):
    ctrl.command = command
    out = []
    for k in range(1, int(seconds / DT) + 1):
        out.append(ctrl.step(k * DT, DT))
    return out


def test_controller_respects_speed_accel_and_jerk_limits():
    L = Limits()
    c = GameController([0, 0, 1.0], 0.0, L)
    ks = run(c, 3.0, (1.0, 0, 0, 0, 0)) + run(c, 3.0, (0, 0, 0, 0, 0))
    v = np.array([k.v for k in ks])
    a = np.array([k.a for k in ks])
    jerk = np.diff(a, axis=0) / DT
    assert np.abs(v[:, 0]).max() <= L.max_speed * 1.02
    assert np.abs(a).max() <= L.max_accel * 1.001
    assert np.abs(jerk).max() <= L.max_jerk * 1.05
    assert v[2999, 0] > 1.5 and np.abs(v[-1]).max() < 0.05          # reaches cruise, then eases to rest


def test_controller_imu_consistency_exact():
    """p and v follow from the recorded acceleration by the scheme the IMU is built on."""
    c = GameController([0, 0, 1.0], 0.0)
    ks = run(c, 1.0, (1.0, 0.5, 0.2, 0.7, 0.3)) + run(c, 1.0, (0, 0, 0, -0.5, 0))
    v = np.zeros(3)
    p = np.array([0, 0, 1.0])
    for k in ks:
        v = v + k.a * DT
        p = p + v * DT
        assert np.allclose(v, k.v, atol=1e-9)
    assert np.allclose(p, ks[-1].p, atol=1e-9)


def test_controller_body_frame_motion_follows_yaw():
    c = GameController([0, 0, 1.0], np.pi / 2)                       # facing +y
    ks = run(c, 2.0, (1.0, 0, 0, 0, 0))
    assert ks[-1].p[1] > 0.5 and abs(ks[-1].p[0]) < 1e-9


def test_controller_collision_stops_at_wall(single_room):
    c = GameController([0, 0, 1.0], 0.0, Limits(), single_room)
    ks = run(c, 6.0, (1.0, 0, 0, 0, 0))
    x = np.array([k.p[0] for k in ks])
    assert x.max() <= 1.9 - c.limits.radius + 1e-6                   # never inside the wall
    assert x.max() > 1.5                                             # but it did get there
    assert abs(ks[-1].v[0]) < 1e-9


def test_trajectory_is_c2_rest_to_rest_and_hits_waypoints():
    wps = [Waypoint((0, 0, 1)), Waypoint((3, 0, 1)), Waypoint((3, 3, 1.5)), Waypoint((0, 0, 1))]
    tr = ScriptedTrajectory(wps, speed=1.0, lead_in=1.0, lead_out=1.0)
    assert np.allclose(tr.step(0.0, DT).v, 0) and np.allclose(tr.step(0.0, DT).a, 0)
    ts = tr.t
    for w, t in zip(wps, ts):
        assert np.allclose(tr.eval(t).p, w.p, atol=1e-9)
    end = tr.eval(tr.duration)
    assert np.allclose(end.v, 0) and np.allclose(end.a, 0)
    # analytic derivatives agree with finite differences
    tt = np.arange(0.1, tr.duration - 0.1, 0.01)
    k = [tr.eval(t) for t in tt]
    p = np.array([x.p for x in k])
    v = np.array([x.v for x in k])
    a = np.array([x.a for x in k])
    assert np.abs(np.gradient(p, 0.01, axis=0)[2:-2] - v[2:-2]).max() < 1e-3
    assert np.abs(np.gradient(v, 0.01, axis=0)[2:-2] - a[2:-2]).max() < 1e-2
    # lead-in is stationary, and finished flags once the run is over
    assert np.allclose(tr.step(0.5, DT).p, wps[0].p)
    assert not tr.finished
    tr.step(tr.total_duration + 0.001, DT)
    assert tr.finished


def test_trajectory_validation():
    with pytest.raises(ValueError):
        ScriptedTrajectory([Waypoint((0, 0, 0))])
    with pytest.raises(ValueError):
        ScriptedTrajectory([Waypoint((0, 0, 0)), Waypoint((1, 0, 0), t=1.0), Waypoint((2, 0, 0), t=0.5)])


@pytest.mark.parametrize("name,scene_fixture", [("two_rooms_loop.yaml", "two_rooms"), ("single_room_loop.yaml", "single_room")])
def test_shipped_trajectories_are_collision_free_and_close_the_loop(name, scene_fixture, request):
    scene = request.getfixturevalue(scene_fixture)
    tr = load_trajectory(ROOT / "trajectories" / name)
    ts = np.arange(0, tr.duration, 0.02)
    pts = np.array([tr.eval(t).p for t in ts])
    bad = [p for p in pts if scene.collides(p, 0.25)]
    assert not bad, f"{len(bad)} samples closer than 0.25 m to a wall/obstacle, first {bad[0]}"
    assert np.linalg.norm(tr.eval(tr.duration).p - tr.eval(0).p) < 1e-3     # returns to its start
    speed = np.linalg.norm([tr.eval(t).v for t in ts], axis=1)
    assert speed.max() < 2.0


def test_rect_loop_generator():
    d = rect_loop((0, 0), (4, 2), 1.0, 0.5, laps=2)
    assert d["waypoints"][0]["p"] == d["waypoints"][-1]["p"]
    assert len(d["waypoints"]) == 1 + 2 * 9
