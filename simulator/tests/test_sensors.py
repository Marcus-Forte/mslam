import numpy as np
import pytest

from msim.engine import kin_to_history_row
from msim.geometry import (GRAVITY, angular_velocity_body, quat_from_rot, rot_from_quat, rot_from_yaw_pitch, so3_log)
from msim.history import TICK_NS, PoseHistory
from msim.motion import ScriptedTrajectory, Waypoint, kin_from_euler
from msim.raycast import BoxCaster
from msim.sensors import ImuModel, LidarModel


class FixedPattern:
    """Every ray along sensor +x: the simplest thing whose per-point range is hand-computable."""

    def directions(self, scan_index, n):
        return np.tile([1.0, 0.0, 0.0], (n, 1))


def expm(w):
    ang = np.linalg.norm(w)
    if ang < 1e-15:
        return np.eye(3)
    k = w / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def fill_history(kin_fn, seconds, start_ns=10**12):
    h = PoseHistory(start_ns, 4096)
    for k in range(int(seconds * 1000) + 1):
        h.append(*kin_to_history_row(kin_fn(k * 1e-3)))
    return h


def const_kin(p, yaw=0.0, pitch=0.0, v=(0, 0, 0), a=(0, 0, 0), yaw_rate=0.0, pitch_rate=0.0, roll=0.0, roll_rate=0.0):
    return kin_from_euler(np.array(p, float), np.array(v, float), np.array(a, float), yaw, yaw_rate, pitch, pitch_rate,
                          roll, roll_rate)


def test_quaternion_roundtrip_and_composition():
    R = rot_from_yaw_pitch(np.array([0.3, -2.0, 3.1]), np.array([0.2, -0.7, 1.0]))
    q = quat_from_rot(R[1])
    assert np.allclose(rot_from_quat(q), R[1])
    assert np.allclose(R[2].T @ R[2], np.eye(3))


@pytest.mark.parametrize("yaw,yr,pitch,pr", [(0.4, 0.7, 0.3, -0.2), (-2.0, -0.3, -0.6, 0.5), (1.0, 0.0, 0.0, 0.0)])
def test_angular_velocity_matches_finite_difference(yaw, yr, pitch, pr):
    dt = 1e-6
    R0 = rot_from_yaw_pitch(yaw, pitch)
    R1 = rot_from_yaw_pitch(yaw + yr * dt, pitch + pr * dt)
    w_fd = so3_log(R0.T @ R1) / dt                      # body-frame rotation increment
    assert np.allclose(angular_velocity_body(np.float64(yaw), np.float64(yr), np.float64(pitch), np.float64(pr)), w_fd, atol=1e-5)


def test_per_point_pose_interpolation_translation(single_room):
    """Sensor moving at 0.5 m/s along +x, rays along +x: range of point i is 1.9 - x(t_i), so the
    scan is skewed by exactly the motion and the points are expressed in the frame at their own time."""
    v = 0.5
    start = 10**12
    h = fill_history(lambda t: const_kin([-0.5 + v * t, 0, 1.0], v=[v, 0, 0]), 0.3, start)
    lidar = LidarModel(BoxCaster(single_room), FixedPattern())
    t0 = start + 100_000_000
    scan = lidar.scan(h, t0, 0)
    t = (scan.t0_ns - start) * 1e-9 + np.arange(9600) / 200_000
    expected_range = 1.9 - (-0.5 + v * t)
    assert scan.xyz.shape == (9600, 3) and scan.hit.all()
    assert np.allclose(scan.xyz[:, 0], expected_range, atol=2e-6)
    assert np.allclose(scan.xyz[:, 1:], 0.0)
    assert scan.xyz[0, 0] - scan.xyz[-1, 0] == pytest.approx(v * 9599 / 200_000, abs=1e-5)
    assert scan.t_end_ns - scan.t0_ns == 9599 * 5000


def test_per_point_pose_interpolation_rotation(single_room):
    """Yaw rate 0.5 rad/s: ray i leaves at yaw(t_i) = w t_i and hits the +x wall at 1.9 / cos(yaw)."""
    w = 0.5
    start = 10**12
    h = fill_history(lambda t: const_kin([0, 0, 1.0], yaw=w * t, yaw_rate=w), 0.3, start)
    scan = LidarModel(BoxCaster(single_room), FixedPattern()).scan(h, start + 100_000_000, 0)
    t = 0.1 + np.arange(9600) / 200_000
    assert np.allclose(scan.xyz[:, 0], 1.9 / np.cos(w * t), atol=3e-5)


def test_misses_keep_the_point_count(single_room):
    # sensor with the ceiling removed would escape; use max range to force misses
    h = fill_history(lambda t: const_kin([0, 0, 1.0]), 0.2)
    lidar = LidarModel(BoxCaster(single_room), FixedPattern(), max_range=1.0)
    scan = lidar.scan(h, 10**12 + 50_000_000, 0)
    assert scan.xyz.shape == (9600, 3) and not scan.hit.any()
    assert np.all(scan.xyz == 0) and np.all(scan.intensity == 0)


def test_hit_intensity_from_the_centre_of_the_room_passes_mslams_filter(single_room):
    """From the centre of single_room every ray hits a wall or the ceiling with cos(incidence) >= 0.56, so with the
    default reflectivities (wall 0.4, ceiling 0.45) every return is >= 20, `min_intensity` of config/mslam.jsonc.
    (The integration config's 50 removes many of them, as it does on the real recordings.)"""
    h = fill_history(lambda t: const_kin([0, 0, 1.0]), 0.2)
    from msim.pattern import RosettePattern
    from msim.reflectivity import ReflectivityModel
    lidar = LidarModel(BoxCaster(single_room), RosettePattern(), reflectivity=ReflectivityModel.from_scene(single_room))
    scan = lidar.scan(h, 10**12 + 50_000_000, 0)
    assert scan.hit.all() and scan.intensity.min() >= 20 and scan.intensity.max() <= 150


def test_range_noise_is_seeded(single_room):
    h = fill_history(lambda t: const_kin([0, 0, 1.0]), 0.2)
    a = LidarModel(BoxCaster(single_room), FixedPattern(), range_noise_std=0.01, rng=np.random.default_rng(5)).scan(h, 10**12 + 5 * 10**7, 0)
    b = LidarModel(BoxCaster(single_room), FixedPattern(), range_noise_std=0.01, rng=np.random.default_rng(5)).scan(h, 10**12 + 5 * 10**7, 0)
    assert np.array_equal(a.xyz, b.xyz)
    assert 0.005 < a.xyz[:, 0].std() < 0.02


def test_history_bounds():
    h = fill_history(lambda t: const_kin([t, 0, 0]), 0.1)
    s = h.sample(np.array([h.start_ns + 1_500_000]))
    assert s.p[0, 0] == pytest.approx(0.0015)
    with pytest.raises(ValueError):
        h.sample(h.latest_ns + 1)


# ---------------- IMU -------------------------------------------------------

def test_static_level_sensor_reads_plus_one_g_on_z():
    h = fill_history(lambda t: const_kin([0, 0, 1.0]), 0.05)
    s = ImuModel().sample(h, h.start_ns + 12_345_678, 0)
    assert np.allclose(s.accel_g, [0, 0, 1.0], atol=1e-12)
    assert np.allclose(s.gyro, 0.0)


def test_static_pitched_sensor_gravity_direction():
    """Looking up by 30 deg: gravity (specific force) in the sensor frame = R^T [0,0,g]."""
    pitch = np.radians(30)
    h = fill_history(lambda t: const_kin([0, 0, 1.0], pitch=pitch), 0.05)
    s = ImuModel().sample(h, h.start_ns + 10_000_000, 0)
    # The accelerometer reads the reaction to gravity, i.e. world UP in the sensor frame. With the nose
    # up the x axis points up, so up has a +sin(pitch) component on x. (mslam recovers pitch =
    # atan2(-ax, az) = -30 deg in its Rx Ry Rz convention, where positive pitch is nose DOWN.)
    assert np.allclose(s.accel_g, [np.sin(pitch), 0, np.cos(pitch)], atol=1e-9)
    assert np.linalg.norm(s.accel_g) == pytest.approx(1.0)


def test_constant_yaw_rate_gives_gyro_z():
    h = fill_history(lambda t: const_kin([0, 0, 1.0], yaw=0.8 * t, yaw_rate=0.8), 0.05)
    s = ImuModel().sample(h, h.start_ns + 20_000_000, 0)
    assert np.allclose(s.gyro, [0, 0, 0.8], atol=1e-12)
    assert np.allclose(s.accel_g, [0, 0, 1.0], atol=1e-12)


def test_imu_noise_defaults_off_and_seeded():
    h = fill_history(lambda t: const_kin([0, 0, 1.0]), 0.05)
    noisy = lambda: ImuModel(accel_noise_density=0.02, gyro_noise_density=0.002, gyro_bias_walk=1e-4,  # noqa: E731
                             rng=np.random.default_rng(9))
    a, b = noisy().sample(h, h.start_ns + 10_000_000, 0), noisy().sample(h, h.start_ns + 10_000_000, 0)
    assert np.array_equal(a.accel_g, b.accel_g) and not np.allclose(a.accel_g, [0, 0, 1.0])


@pytest.mark.parametrize("roll", [(0.0, 0.0, 0.0), (0.0, 0.5, -0.4)])
def test_imu_integrates_back_to_the_trajectory(roll):
    """A scripted accelerating, turning, pitching flight: double-integrating the IMU (attitude from the
    integrated gyro) must reproduce the true position."""
    traj = ScriptedTrajectory([Waypoint((0, 0, 1.0), yaw=0.0, roll=roll[0]), Waypoint((2, 0, 1.2), pitch=0.2, roll=roll[1]),
                               Waypoint((3, 2, 1.0), yaw=1.2, pitch=-0.1, roll=roll[2]), Waypoint((1, 3, 1.0))],
                              speed=1.0, lead_in=0.5, lead_out=0.5)
    T = traj.total_duration
    h = PoseHistory(10**12, 16384)
    h.append(*kin_to_history_row(traj.step(0.0, 1e-3)))
    for k in range(1, int(T * 1000) + 1):
        h.append(*kin_to_history_row(traj.step(k * 1e-3, 1e-3)))
    imu = ImuModel()
    dt = 0.005
    n = int((T - 0.05) / dt)
    R = h.sample(h.start_ns).R[0].copy()
    p = h.sample(h.start_ns).p[0].copy()
    v = h.sample(h.start_ns).v[0].copy()
    t_prev = h.start_ns
    last = None
    for i in range(n):
        t = h.start_ns + 1_300_000 + i * 5_000_000
        s = imu.sample(h, t, i)
        if last is not None:
            dts = (t - t_prev) * 1e-9
            # attitude by the exponential map of the mean gyro; the acceleration uses the mid-interval attitude
            w = 0.5 * (s.gyro + last.gyro) * dts
            R_mid = R @ expm(0.5 * w)
            R = R @ expm(w)
            a_w = R_mid @ (0.5 * (s.accel_g + last.accel_g) * GRAVITY) + np.array([0, 0, -GRAVITY])
            p = p + v * dts + 0.5 * a_w * dts**2
            v = v + a_w * dts
        last, t_prev = s, t
    truth = h.sample(t_prev)
    assert np.linalg.norm(p - truth.p[0]) < 5e-3, (p, truth.p[0])
    assert np.linalg.norm(v - truth.v[0]) < 5e-3
    assert np.linalg.norm(so3_log(truth.R[0].T @ R)) < 5e-3
