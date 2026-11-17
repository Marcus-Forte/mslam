"""Follow view: the followed part of the pose stays still on screen, nothing jumps when the mode changes."""

import math
import unittest

import numpy as np

from tests import helpers  # noqa: F401  (puts the client directory on sys.path)

from follow import FollowMode, FollowView, apply, compose, heading, inverse  # noqa: E402
from geometry import euler_xyz_to_wxyz  # noqa: E402


def pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
    """mslam's pose convention (R = Rx Ry Rz), as the viewer converts it."""
    q = euler_xyz_to_wxyz(roll, pitch, yaw).astype(np.float64)
    return q / np.linalg.norm(q), np.array([x, y, z])


def wiggle(k: int):
    """A hand-held sensor walking a curve: moving, turning, rolling and pitching."""
    t = 0.05 * k
    return pose(2.0 * math.sin(t), 1.5 * t, 0.1 * math.sin(3 * t), 0.2 * math.sin(7 * t), 0.15 * math.cos(5 * t), t)


def on_screen_pose(view: FollowView, p):
    return compose(view.root, p)


def rotation(q: np.ndarray) -> np.ndarray:
    return np.stack([apply((q, np.zeros(3)), e) for e in np.eye(3)], axis=1)


class FollowViewTest(unittest.TestCase):
    def test_off_never_moves_the_world(self) -> None:
        view = FollowView()
        for k in range(20):
            root = view.update(wiggle(k))
            np.testing.assert_allclose(root[0], [1, 0, 0, 0])
            np.testing.assert_allclose(root[1], 0.0)

    def test_position_keeps_the_sensor_still_and_the_view_orientation(self) -> None:
        view = FollowView()
        view.set_mode(FollowMode.POSITION, wiggle(0))
        start = view.on_screen(wiggle(0)[1])
        for k in range(1, 50):
            p = wiggle(k)
            root = view.update(p)
            np.testing.assert_allclose(view.on_screen(p[1]), start, atol=1e-12)
            np.testing.assert_allclose(root[0], [1, 0, 0, 0])  # the world only translates

    def test_position_yaw_follows_the_heading_but_not_roll_and_pitch(self) -> None:
        view = FollowView()
        view.set_mode(FollowMode.POSITION_YAW, wiggle(0))
        start_heading = heading(on_screen_pose(view, wiggle(0))[0])
        for k in range(1, 50):
            p = wiggle(k)
            view.update(p)
            screen = on_screen_pose(view, p)
            np.testing.assert_allclose(screen[1], wiggle(0)[1], atol=1e-12)
            self.assertAlmostEqual(heading(screen[0]), start_heading, places=9)
            # The world stays upright: its z axis is drawn vertical.
            np.testing.assert_allclose(rotation(view.root[0])[:, 2], [0, 0, 1], atol=1e-12)

    def test_full_pose_keeps_the_whole_sensor_frame_still(self) -> None:
        view = FollowView()
        view.set_mode(FollowMode.FULL, wiggle(0))
        for k in range(1, 50):
            p = wiggle(k)
            view.update(p)
            screen = on_screen_pose(view, p)
            np.testing.assert_allclose(rotation(screen[0]), rotation(wiggle(0)[0]), atol=1e-12)
            np.testing.assert_allclose(screen[1], wiggle(0)[1], atol=1e-12)

    def test_switching_modes_never_jumps(self) -> None:
        view = FollowView()
        modes = [FollowMode.POSITION, FollowMode.OFF, FollowMode.FULL, FollowMode.POSITION_YAW, FollowMode.POSITION]
        k = 0
        for mode in modes:
            before = view.update(wiggle(k))
            view.set_mode(mode, wiggle(k))
            after = view.update(wiggle(k))
            for p in ([0, 0, 0], [5, -3, 2], wiggle(k)[1]):
                np.testing.assert_allclose(apply(after, p), apply(before, p), atol=1e-9)
            for _ in range(15):
                k += 1
                view.update(wiggle(k))

    def test_off_after_following_freezes_the_world(self) -> None:
        view = FollowView()
        view.set_mode(FollowMode.POSITION_YAW, wiggle(0))
        frozen = view.update(wiggle(10))
        view.set_mode(FollowMode.OFF, wiggle(10))
        for k in range(11, 30):
            root = view.update(wiggle(k))
            np.testing.assert_allclose(root[1], frozen[1])
            np.testing.assert_allclose(root[0], frozen[0])

    def test_vertical_sensor_keeps_the_last_heading(self) -> None:
        view = FollowView()
        view.set_mode(FollowMode.POSITION_YAW, pose(0, 0, 0, yaw=0.7))
        root = view.update(pose(1, 0, 0, yaw=0.7))
        up = pose(1, 0, 0, pitch=-math.pi / 2)  # x axis straight up
        self.assertIsNone(heading(up[0]))
        np.testing.assert_allclose(view.update(up)[0], root[0], atol=1e-12)

    def test_inverse(self) -> None:
        p = wiggle(7)
        q, t = compose(p, inverse(p))
        np.testing.assert_allclose(abs(q[0]), 1.0, atol=1e-12)
        np.testing.assert_allclose(t, 0.0, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
