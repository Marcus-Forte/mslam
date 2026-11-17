"""Kinematic motion sources: a game-style controller and scripted trajectories.

Both produce a `Kin` state per 1 ms tick: position, world velocity and
acceleration, attitude `R` (body-to-world) and body-frame angular velocity `w`.
The IMU is computed from exactly these quantities, so the accelerometer and gyro
are consistent with the LiDAR poses by construction.

* `GameController` has full 3D attitude (roll, pitch, yaw, no limits, any
  orientation including inverted). Three body-frame angular-rate channels
  (about x, y, z) are jerk-limited; `R <- R Exp(w dt)` with `w` the channel
  velocities AFTER the update, which is the `w` reported in `Kin`. Likewise the
  translation channels update the velocity first and the position integrates the
  NEW velocity, with the acceleration defined as the velocity difference over the
  tick, so integrating the IMU reproduces the pose exactly (no scheme mismatch).
  Translation frames: `level` (default) commands forward/left along the horizontal
  heading of the body x axis and up along world z; `body` commands along the body
  axes (flying).
* `ScriptedTrajectory` is a C2 quintic-Hermite spline through waypoints with
  analytic velocity and acceleration (yaw, pitch-up and roll Euler angles, see
  `geometry`); it starts and ends at rest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple, Protocol

import numpy as np
import yaml

from .geometry import angular_velocity_body, exp_so3, orthonormalize, rot_from_yaw_pitch, rot_z


@dataclass
class Kin:
    p: np.ndarray    # (3,) world position
    v: np.ndarray    # (3,) world velocity
    a: np.ndarray    # (3,) world acceleration (gravity not included)
    R: np.ndarray    # (3,3) body-to-world rotation
    w: np.ndarray    # (3,) body-frame angular velocity, rad/s


def kin_from_euler(p, v, a, yaw, yaw_rate, pitch, pitch_rate, roll=0.0, roll_rate=0.0) -> Kin:
    """Kin from yaw / pitch-up / roll Euler angles (R = Rz(yaw) Ry(-pitch) Rx(roll)) and their rates."""
    R = rot_from_yaw_pitch(yaw, pitch, roll)
    w = angular_velocity_body(np.float64(yaw), np.float64(yaw_rate), np.float64(pitch), np.float64(pitch_rate),
                              np.float64(roll), np.float64(roll_rate))
    return Kin(np.asarray(p, float), np.asarray(v, float), np.asarray(a, float), R, w)


class MotionSource(Protocol):
    finished: bool

    def initial(self) -> Kin: ...
    def step(self, t: float, dt: float) -> Kin: ...


# ---------------------------------------------------------------------------
# Game controller
# ---------------------------------------------------------------------------

@dataclass
class Limits:
    max_speed: float = 2.0       # m/s horizontal (forward, left)
    max_vspeed: float = 1.2      # m/s up
    max_accel: float = 2.0       # m/s^2
    max_jerk: float = 8.0        # m/s^3
    max_yaw_rate: float = 1.2    # rad/s
    max_yaw_accel: float = 3.0   # rad/s^2
    max_yaw_jerk: float = 15.0   # rad/s^3
    max_pitch_rate: float = 0.8
    max_pitch_accel: float = 3.0
    max_pitch_jerk: float = 15.0
    max_roll_rate: float = 0.8
    max_roll_accel: float = 3.0
    max_roll_jerk: float = 15.0
    level_gain: float = 2.0      # 1/s, proportional gain of the level command
    max_heading_rate: float = 3.0  # rad/s, how fast the `level` translation heading may follow the x axis
    tau: float = 0.3             # s, time constant towards the commanded velocity
    radius: float = 0.25         # m, collision sphere


class Command(NamedTuple):
    """Replaced as a whole by the UI thread (atomic reference swap). Components in [-1, 1]."""

    forward: float = 0.0
    left: float = 0.0
    up: float = 0.0
    yaw_rate: float = 0.0     # + = turn left (right-hand rotation about body z)
    pitch_rate: float = 0.0   # + = look up (nose up)
    roll_rate: float = 0.0    # + = roll right (right-hand rotation about body x, left side up)
    level: bool = False       # level the sensor (z axis back to world up, heading kept); overrides roll/pitch


class _Axis:
    """Velocity channel with acceleration and jerk limits."""

    def __init__(self, vmax: float, amax: float, jmax: float, tau: float) -> None:
        self.vmax, self.amax, self.jmax, self.tau = vmax, amax, jmax, tau
        self.v = 0.0
        self.a = 0.0

    def step(self, command: float, dt: float) -> None:
        target = max(-1.0, min(1.0, command)) * self.vmax
        a_des = max(-self.amax, min(self.amax, (target - self.v) / self.tau))
        self.a += max(-self.jmax * dt, min(self.jmax * dt, a_des - self.a))
        self.a = max(-self.amax, min(self.amax, self.a))
        self.v += self.a * dt

    def zero(self) -> None:
        self.v = self.a = 0.0


def log_so3(R: np.ndarray) -> np.ndarray:
    """Rotation vector of R, robust near pi."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-9:
        return 0.5 * v
    if math.pi - ang > 1e-3:
        return v * ang / (2.0 * math.sin(ang))
    # near pi: axis from the symmetric part (R + I)/2 = a a^T, sign from v
    M = (R + np.eye(3)) / 2.0
    k = int(np.argmax(np.diag(M)))
    axis = M[:, k] / math.sqrt(max(M[k, k], 1e-12))
    if v @ axis < 0:
        axis = -axis
    return axis * ang


class GameController:
    finished = False

    def __init__(self, start_position, start_yaw: float, limits: Limits | None = None, scene=None,
                 frame_mode: str = "level") -> None:
        self.limits = limits or Limits()
        self.scene = scene
        self._start = (np.array(start_position, float), float(start_yaw))
        L = self.limits
        self._fwd = _Axis(L.max_speed, L.max_accel, L.max_jerk, L.tau)
        self._left = _Axis(L.max_speed, L.max_accel, L.max_jerk, L.tau)
        self._up = _Axis(L.max_vspeed, L.max_accel, L.max_jerk, L.tau)
        # body-frame angular rates about x (roll), y (pitch, + = nose DOWN) and z (yaw)
        self._wx = _Axis(L.max_roll_rate, L.max_roll_accel, L.max_roll_jerk, 0.2)
        self._wy = _Axis(L.max_pitch_rate, L.max_pitch_accel, L.max_pitch_jerk, 0.2)
        self._wz = _Axis(L.max_yaw_rate, L.max_yaw_accel, L.max_yaw_jerk, 0.2)
        if frame_mode not in ("level", "body"):
            raise ValueError("frame_mode must be 'level' or 'body'")
        self.frame_mode = frame_mode
        self.command: Command | tuple = Command()
        self._steps = 0
        self.reset()

    # -- state ---------------------------------------------------------------
    def reset(self) -> None:
        """Back to the start pose and attitude, at rest; the translation mode is kept."""
        self.p = self._start[0].copy()
        self.R = rot_z(self._start[1])
        self._heading = self._start[1]
        self.v = np.zeros(3)
        for ax in (self._fwd, self._left, self._up, self._wx, self._wy, self._wz):
            ax.zero()

    def toggle_frame_mode(self) -> str:
        self.frame_mode = "body" if self.frame_mode == "level" else "level"
        return self.frame_mode

    def initial(self) -> Kin:
        return Kin(self.p.copy(), np.zeros(3), np.zeros(3), self.R.copy(), np.zeros(3))

    def _level_rates(self) -> np.ndarray:
        """Body rates (rad/s) that bring the attitude to Rz(heading): z up, heading of the x axis kept."""
        R_d = rot_z(self._heading)
        e = log_so3(self.R.T @ R_d)
        return self.limits.level_gain * e

    def _update_heading(self, dt: float) -> None:
        """Heading of the `level` translation frame: follows the horizontal projection of the x axis, continuously.

        The projection flips by 180 degrees when x passes through vertical (pitching over the top), and it
        swings fast near vertical. Taken raw, the translation velocity would flip within one tick (a 4000 m/s^2
        spike in the IMU at 2 m/s). So the heading keeps continuity through vertical (the flipped projection is
        used when it is the nearer one) and moves at most `max_heading_rate`; it is kept while x is vertical.
        """
        xh = self.R[:2, 0]
        if math.hypot(xh[0], xh[1]) <= 1e-3:
            return
        target = math.atan2(xh[1], xh[0])
        delta = math.remainder(target - self._heading, 2.0 * math.pi)
        if abs(delta) > 0.5 * math.pi:                 # over the top: the x projection now points backwards
            delta = math.remainder(delta + math.pi, 2.0 * math.pi)
        step = self.limits.max_heading_rate * dt
        self._heading = math.remainder(self._heading + max(-step, min(step, delta)), 2.0 * math.pi)

    def _translation_basis(self) -> np.ndarray:
        if self.frame_mode == "body":
            return self.R
        return rot_z(self._heading)

    def step(self, t: float, dt: float) -> Kin:
        cmd = self.command if isinstance(self.command, Command) else Command(*self.command)
        L = self.limits
        # ---- attitude: channels first, then R <- R Exp(w dt) with the updated w
        if cmd.level:
            rates = self._level_rates()
            self._wx.step(rates[0] / L.max_roll_rate, dt)
            self._wy.step(rates[1] / L.max_pitch_rate, dt)
        else:
            self._wx.step(cmd.roll_rate, dt)
            self._wy.step(-cmd.pitch_rate, dt)        # nose up = negative rotation about y
        if cmd.level:
            self._wz.step(rates[2] / L.max_yaw_rate, dt)
        else:
            self._wz.step(cmd.yaw_rate, dt)
        w = np.array([self._wx.v, self._wy.v, self._wz.v])
        self.R = self.R @ exp_so3(w * dt)
        self._steps += 1
        if self._steps % 64 == 0:
            self.R = orthonormalize(self.R)
        self._update_heading(dt)

        # ---- translation
        self._fwd.step(cmd.forward, dt)
        self._left.step(cmd.left, dt)
        self._up.step(cmd.up, dt)
        M = self._translation_basis()
        chan = np.array([self._fwd.v, self._left.v, self._up.v])
        v_new = M @ chan
        p_new = self.p + v_new * dt
        if self.scene is not None:
            # Slide: resolve axis by axis in the world frame and drop the velocity component that hit something.
            p_try = self.p.copy()
            blocked = False
            for ax in range(3):
                cand = p_try.copy()
                cand[ax] = p_new[ax]
                if self.scene.collides(cand, L.radius):
                    v_new[ax] = 0.0
                    blocked = True
                else:
                    p_try = cand
            p_new = p_try
            if blocked:
                # Project the blocked world velocity back onto the channels (M is orthonormal) and clear their
                # accelerations, so nothing winds up against the obstacle.
                c = M.T @ v_new
                self._fwd.v, self._left.v, self._up.v = float(c[0]), float(c[1]), float(c[2])
                self._fwd.a = self._left.a = self._up.a = 0.0
        a = (v_new - self.v) / dt
        self.v, self.p = v_new, p_new
        return Kin(self.p.copy(), self.v.copy(), a, self.R.copy(), w)


# ---------------------------------------------------------------------------
# Scripted trajectory
# ---------------------------------------------------------------------------

def _quintic(x0, v0, a0, x1, v1, a1, T, s):
    """Quintic Hermite on [0, T] at s in [0, T]: returns (x, v, a) with C2 boundary data."""
    u = s / T
    # basis for x(u): p0, v0*T, a0*T^2/2, p1, v1*T, a1*T^2/2
    b0 = 1 - 10 * u**3 + 15 * u**4 - 6 * u**5
    b1 = u - 6 * u**3 + 8 * u**4 - 3 * u**5
    b2 = 0.5 * u**2 - 1.5 * u**3 + 1.5 * u**4 - 0.5 * u**5
    b3 = 10 * u**3 - 15 * u**4 + 6 * u**5
    b4 = -4 * u**3 + 7 * u**4 - 3 * u**5
    b5 = 0.5 * u**3 - u**4 + 0.5 * u**5
    x = b0 * x0 + b1 * v0 * T + b2 * a0 * T**2 + b3 * x1 + b4 * v1 * T + b5 * a1 * T**2
    d0 = -30 * u**2 + 60 * u**3 - 30 * u**4
    d1 = 1 - 18 * u**2 + 32 * u**3 - 15 * u**4
    d2 = u - 4.5 * u**2 + 6 * u**3 - 2.5 * u**4
    d3 = 30 * u**2 - 60 * u**3 + 30 * u**4
    d4 = -12 * u**2 + 28 * u**3 - 15 * u**4
    d5 = 1.5 * u**2 - 4 * u**3 + 2.5 * u**4
    v = (d0 * x0 + d1 * v0 * T + d2 * a0 * T**2 + d3 * x1 + d4 * v1 * T + d5 * a1 * T**2) / T
    e0 = -60 * u + 180 * u**2 - 120 * u**3
    e1 = -36 * u + 96 * u**2 - 60 * u**3
    e2 = 1 - 9 * u + 18 * u**2 - 10 * u**3
    e3 = 60 * u - 180 * u**2 + 120 * u**3
    e4 = -24 * u + 84 * u**2 - 60 * u**3
    e5 = 3 * u - 12 * u**2 + 10 * u**3
    a = (e0 * x0 + e1 * v0 * T + e2 * a0 * T**2 + e3 * x1 + e4 * v1 * T + e5 * a1 * T**2) / T**2
    return x, v, a


class _Channel:
    """One scalar quintic spline through (t_k, x_k) with Catmull-Rom velocities; rest at both ends."""

    def __init__(self, t: np.ndarray, x: np.ndarray, stops: np.ndarray) -> None:
        self.t, self.x = t, x
        n = len(t)
        v = np.zeros(n)
        for k in range(1, n - 1):
            v[k] = 0.0 if stops[k] else (x[k + 1] - x[k - 1]) / (t[k + 1] - t[k - 1])
        a = np.zeros(n)
        for k in range(1, n - 1):
            a[k] = 0.0 if stops[k] else (v[k + 1] - v[k - 1]) / (t[k + 1] - t[k - 1])
        self.v, self.a = v, a

    def eval(self, t: float) -> tuple[float, float, float]:
        tt = min(max(t, self.t[0]), self.t[-1])
        k = int(np.searchsorted(self.t, tt, side="right")) - 1
        k = min(max(k, 0), len(self.t) - 2)
        T = self.t[k + 1] - self.t[k]
        x, v, a = _quintic(self.x[k], self.v[k], self.a[k], self.x[k + 1], self.v[k + 1], self.a[k + 1], T, tt - self.t[k])
        if t <= self.t[0] or t >= self.t[-1]:
            v = a = 0.0
        return float(x), float(v), float(a)


@dataclass
class Waypoint:
    p: tuple[float, float, float]
    yaw: float | None = None   # rad; None = follow the direction of travel
    pitch: float = 0.0
    roll: float = 0.0          # rad
    t: float | None = None     # s; None = derived from the speed
    stop: bool = False


class ScriptedTrajectory:
    """Spline through waypoints; stationary for `lead_in` s before and `lead_out` s after."""

    def __init__(self, waypoints: list[Waypoint], speed: float = 0.8, lead_in: float = 1.0,
                 lead_out: float = 1.0) -> None:
        if len(waypoints) < 2:
            raise ValueError("a trajectory needs at least 2 waypoints")
        self.lead_in, self.lead_out = lead_in, lead_out
        P = np.array([w.p for w in waypoints], float)
        n = len(P)
        # times from the speed where not given
        t = np.zeros(n)
        for k in range(1, n):
            if waypoints[k].t is not None:
                t[k] = waypoints[k].t
            else:
                d = float(np.linalg.norm(P[k] - P[k - 1]))
                # quintic rest-to-rest segments are slower than the nominal speed; leave margin
                t[k] = t[k - 1] + max(1.5 * d / speed, 0.5)
        if any(t[k] <= t[k - 1] for k in range(1, n)):
            raise ValueError("waypoint times must be strictly increasing")
        stops = np.array([w.stop for w in waypoints])
        # headings from the direction of travel (Catmull-Rom tangent), for waypoints without a yaw
        tan = np.zeros((n, 2))
        for k in range(n):
            a, b = max(k - 1, 0), min(k + 1, n - 1)
            tan[k] = (P[b, :2] - P[a, :2])
        yaw = np.zeros(n)
        last = 0.0
        for k in range(n):
            if waypoints[k].yaw is not None:
                last = waypoints[k].yaw
            elif np.hypot(*tan[k]) > 1e-9:
                last = math.atan2(tan[k][1], tan[k][0])
            yaw[k] = last
        yaw = np.unwrap(yaw)
        self.t = t
        self.duration = float(t[-1])
        self.total_duration = lead_in + self.duration + lead_out
        self.finished = False
        self._ch_p = [_Channel(t, P[:, i], stops) for i in range(3)]
        self._ch_yaw = _Channel(t, yaw, stops)
        self._ch_pitch = _Channel(t, np.array([w.pitch for w in waypoints]), stops)
        self._ch_roll = _Channel(t, np.array([w.roll for w in waypoints]), stops)

    def eval(self, t_traj: float) -> Kin:
        p, v, a = zip(*(c.eval(t_traj) for c in self._ch_p))
        yaw, yr, _ = self._ch_yaw.eval(t_traj)
        pit, pr, _ = self._ch_pitch.eval(t_traj)
        rol, rr, _ = self._ch_roll.eval(t_traj)
        return kin_from_euler(np.array(p), np.array(v), np.array(a), yaw, yr, pit, pr, rol, rr)

    def initial(self) -> Kin:
        return self.eval(0.0)

    def step(self, t: float, dt: float) -> Kin:
        """`t` is the simulated time since the start of the run (includes the lead-in)."""
        if t >= self.total_duration:
            self.finished = True
        return self.eval(t - self.lead_in)


def load_trajectory(path: str | Path) -> ScriptedTrajectory:
    """YAML: {speed, lead_in, lead_out, waypoints: [{p: [x,y,z], yaw_deg?, pitch_deg?, roll_deg?, t?, stop?}]}"""
    with open(path) as f:
        d = yaml.safe_load(f)
    wps = []
    for w in d["waypoints"]:
        wps.append(Waypoint(tuple(float(c) for c in w["p"]),
                            math.radians(float(w["yaw_deg"])) if "yaw_deg" in w else None,
                            math.radians(float(w.get("pitch_deg", 0.0))),
                            math.radians(float(w.get("roll_deg", 0.0))),
                            float(w["t"]) if "t" in w else None, bool(w.get("stop", False))))
    return ScriptedTrajectory(wps, float(d.get("speed", 0.8)), float(d.get("lead_in", 1.0)),
                              float(d.get("lead_out", 1.0)))


def rect_loop(center, size, z: float = 1.0, speed: float = 0.8, laps: int = 1, corner: float = 0.8) -> dict:
    """Waypoint dict of a rectangle loop with chamfered corners, counter-clockwise.

    Starts at the middle of the south edge and ends at exactly the same point, at rest,
    heading along +x at both ends once the yaw is unwrapped (one full turn per lap).
    """
    cx, cy = center
    hw, hh = size[0] / 2.0, size[1] / 2.0
    c = min(corner, hw, hh)
    corners = [(hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]
    dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    pts = [(cx, cy - hh)]
    for _ in range(laps):
        for (px, py), (dx, dy), (nx, ny) in zip(corners, dirs, dirs[1:] + dirs[:1]):
            pts.append((cx + px - dx * c, cy + py - dy * c))
            pts.append((cx + px + nx * c, cy + py + ny * c))
        pts.append((cx, cy - hh))
    return {"speed": speed, "lead_in": 1.0, "lead_out": 1.0,
            "waypoints": [{"p": [round(x, 3), round(y, 3), z]} for x, y in pts]}
