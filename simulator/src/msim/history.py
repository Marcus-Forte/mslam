"""High-rate pose history ring buffer with vectorised interpolation.

The kinematic body is stepped at a fixed rate (1 kHz) and every step is stored.
A LiDAR point fired at an arbitrary nanosecond is looked up by interpolating
between the two surrounding steps: position, velocity and acceleration
linearly, attitude by normalised lerp of the quaternion (error far below
1e-6 rad at 1 ms spacing).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import nlerp, rot_from_quat

TICK_NS = 1_000_000  # 1 kHz


@dataclass
class Sample:
    p: np.ndarray        # (N,3) world position
    R: np.ndarray        # (N,3,3) body-to-world
    v: np.ndarray        # (N,3) world velocity
    a: np.ndarray        # (N,3) world acceleration (kinematic, gravity not included)
    w: np.ndarray        # (N,3) body-frame angular velocity
    q: np.ndarray        # (N,4) quaternion (w,x,y,z)


class PoseHistory:
    def __init__(self, start_ns: int, capacity: int = 8192) -> None:
        self.start_ns = int(start_ns)
        self.capacity = capacity
        self.count = 0  # ticks appended so far; tick k is at start_ns + k * TICK_NS
        self._p = np.zeros((capacity, 3))
        self._q = np.zeros((capacity, 4))
        self._v = np.zeros((capacity, 3))
        self._a = np.zeros((capacity, 3))
        self._w = np.zeros((capacity, 3))

    @property
    def latest_ns(self) -> int:
        return self.start_ns + (self.count - 1) * TICK_NS

    @property
    def oldest_ns(self) -> int:
        return self.start_ns + max(0, self.count - self.capacity) * TICK_NS

    def append(self, p, q, v, a, w) -> None:
        i = self.count % self.capacity
        self._p[i], self._q[i], self._v[i], self._a[i], self._w[i] = p, q, v, a, w
        self.count += 1

    def sample(self, t_ns) -> Sample:
        """Interpolate at the given times (int array or scalar, ns). Must lie in [oldest, latest]."""
        t = np.atleast_1d(np.asarray(t_ns, dtype=np.int64))
        if t.min() < self.oldest_ns or t.max() > self.latest_ns:
            raise ValueError(
                f"time outside the history: [{t.min()}, {t.max()}] vs [{self.oldest_ns}, {self.latest_ns}]")
        rel = t - self.start_ns
        k0 = rel // TICK_NS
        frac = (rel - k0 * TICK_NS) / TICK_NS
        k1 = np.minimum(k0 + 1, self.count - 1)
        i0, i1 = k0 % self.capacity, k1 % self.capacity
        f = frac[:, None]
        q = nlerp(self._q[i0], self._q[i1], frac)
        return Sample(
            p=self._p[i0] * (1 - f) + self._p[i1] * f,
            R=rot_from_quat(q),
            v=self._v[i0] * (1 - f) + self._v[i1] * f,
            a=self._a[i0] * (1 - f) + self._a[i1] * f,
            w=self._w[i0] * (1 - f) + self._w[i1] * f,
            q=q,
        )
