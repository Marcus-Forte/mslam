"""Follow the SLAM pose the way rviz does: by moving the scene, never the browser's camera.

Every visual (map, scan, correspondences, pose axes) lives under one root frame, `/world`. Following sets that
root to `A * F(t)^-1`, where `F(t)` is the followed part of the pose and `A` is fixed when following starts, so
the followed frame stays where it is on screen and the rest of the world moves around it. The camera is left
alone, so orbiting, panning and zooming work while following and are never overwritten.

Modes (rviz equivalents in brackets):

* `Off`: the root stays where it is (no jump when following stops).
* `Position` [Orbit view with a target frame]: only the position is followed; the view keeps its orientation.
* `Position + yaw` [ThirdPersonFollower]: position and heading (yaw of the sensor's x axis about the world z);
  roll and pitch of a hand-held sensor do not swing the view.
* `Full pose`: position and full attitude (first person; shakes with every roll and pitch of the sensor).

Why not move the camera: viser animates a camera position set from the server over 50 ms but applies the look-at
at once, so the view wobbles, and the camera state the browser reports back lags behind what the server set, so
an offset read from it drags the view behind the sensor.

Pure numpy, no viser: transforms are `(wxyz, position)` pairs, rotation `R(wxyz)`, `p_parent = R p + t`.
"""

import math
from enum import Enum

import numpy as np

Transform = tuple[np.ndarray, np.ndarray]
"""(wxyz unit quaternion, position), float64."""

IDENTITY: Transform = (np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))


class FollowMode(str, Enum):
    OFF = "Off"
    POSITION = "Position"
    POSITION_YAW = "Position + yaw"
    FULL = "Full pose"


FOLLOW_MODES = tuple(m.value for m in FollowMode)


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def _qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    u = q[1:]
    t = 2.0 * np.cross(u, v)
    return v + q[0] * t + np.cross(u, t)


def _qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def compose(a: Transform, b: Transform) -> Transform:
    """a * b."""
    q = _qmul(a[0], b[0])
    return q / np.linalg.norm(q), a[1] + _qrot(a[0], b[1])


def inverse(a: Transform) -> Transform:
    qi = _qconj(a[0] / np.linalg.norm(a[0]))
    return qi, -_qrot(qi, a[1])


def apply(a: Transform, p: np.ndarray) -> np.ndarray:
    return a[1] + _qrot(a[0], np.asarray(p, float))


def heading(wxyz: np.ndarray) -> float | None:
    """Yaw of the body x axis about the world z, or None when x is (nearly) vertical."""
    x = _qrot(np.asarray(wxyz, float), np.array([1.0, 0.0, 0.0]))
    if math.hypot(x[0], x[1]) < 1e-3:
        return None
    return math.atan2(x[1], x[0])


def yaw_quaternion(yaw: float) -> np.ndarray:
    return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


class FollowView:
    """The root transform for a stream of poses and mode changes. Not thread safe: call from one thread."""

    def __init__(self) -> None:
        self.mode = FollowMode.OFF
        self.root: Transform = IDENTITY
        self._anchor: Transform = IDENTITY
        self._heading = 0.0

    def _followed(self, pose: Transform) -> Transform:
        q, p = np.asarray(pose[0], float), np.asarray(pose[1], float)
        if self.mode is FollowMode.POSITION:
            return IDENTITY[0], p
        if self.mode is FollowMode.POSITION_YAW:
            h = heading(q)
            if h is not None:
                self._heading = h
            return yaw_quaternion(self._heading), p
        return q / np.linalg.norm(q), p

    def set_mode(self, mode: FollowMode, pose: Transform) -> None:
        """Switch mode at `pose` without moving anything on screen."""
        self.mode = FollowMode(mode)
        if self.mode is FollowMode.OFF:
            return
        # A = root * F(t0), so root = A * F(t0)^-1 is unchanged now.
        self._anchor = compose(self.root, self._followed(pose))

    def update(self, pose: Transform) -> Transform:
        """Root transform for the latest pose."""
        if self.mode is not FollowMode.OFF:
            self.root = compose(self._anchor, inverse(self._followed(pose)))
        return self.root

    def on_screen(self, p: np.ndarray) -> np.ndarray:
        """Where a world point is drawn (in the browser's scene frame)."""
        return apply(self.root, p)
