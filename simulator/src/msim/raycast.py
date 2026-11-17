"""Ray casters against a `Scene`.

`BoxCaster` is the fast path: a vectorised slab test of N rays against the
scene's oriented boxes in numpy, with a different origin per ray (needed because
every LiDAR point is fired from the sensor pose at its own time). MuJoCo's
`mj_multiRay` shares one origin between rays, so it cannot do that in one call;
`MujocoCaster` therefore loops `mj_ray` per point and is kept as the
cross-check (tests) and as `--caster mujoco`.
"""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np

from .scene import Scene


class Caster(Protocol):
    def cast(self, origins: np.ndarray, dirs: np.ndarray, min_range: float, max_range: float
             ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Distance along each unit direction (inf for a miss), |cos| of the incidence angle, and the index
        of the hit box in `Scene.boxes` (-1 for a miss)."""
        ...


def _face_cos(center, half, c, s, p, d) -> float:
    """|cos| of the incidence angle of direction `d` on the face of an oriented box that contains point `p`."""
    rx, ry, rz = p[0] - center[0], p[1] - center[1], p[2] - center[2]
    lp = np.array([c * rx + s * ry, -s * rx + c * ry, rz])
    ld = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    axis = int(np.argmax(np.abs(lp) / half))
    return float(abs(ld[axis]))


class BoxCaster:
    def __init__(self, scene: Scene) -> None:
        bs = scene.boxes
        self.center = np.array([b.center for b in bs], dtype=np.float64)
        self.half = np.array([b.half for b in bs], dtype=np.float64)
        self.cos = np.array([math.cos(b.yaw) for b in bs])
        self.sin = np.array([math.sin(b.yaw) for b in bs])
        self.n_surfaces = len(bs)

    def cast(self, origins, dirs, min_range=0.1, max_range=40.0):
        n = origins.shape[0]
        best = np.full(n, np.inf)
        best_cos = np.zeros(n)
        best_idx = np.full(n, -1, dtype=np.int64)
        ox, oy, oz = origins[:, 0], origins[:, 1], origins[:, 2]
        dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            for k in range(self.center.shape[0]):
                c, s = self.cos[k], self.sin[k]
                cx, cy, cz = self.center[k]
                hx, hy, hz = self.half[k]
                rx, ry, rz = ox - cx, oy - cy, oz - cz
                # world -> box frame: R(yaw)^T
                lox, loy = c * rx + s * ry, -s * rx + c * ry
                ldx, ldy = c * dx + s * dy, -s * dx + c * dy
                ix, iy, iz = 1.0 / ldx, 1.0 / ldy, 1.0 / dz
                tx1, tx2 = (-hx - lox) * ix, (hx - lox) * ix
                ty1, ty2 = (-hy - loy) * iy, (hy - loy) * iy
                tz1, tz2 = (-hz - rz) * iz, (hz - rz) * iz
                txn, txf = np.minimum(tx1, tx2), np.maximum(tx1, tx2)
                tyn, tyf = np.minimum(ty1, ty2), np.maximum(ty1, ty2)
                tzn, tzf = np.minimum(tz1, tz2), np.maximum(tz1, tz2)
                tn = np.maximum(np.maximum(txn, tyn), tzn)
                tf = np.minimum(np.minimum(txf, tyf), tzf)
                # An origin inside the box (tn < 0) is not a hit of this box. A hit nearer than min_range
                # still hides what is behind it; it is rejected below, after the nearest box is known.
                hit = (tn <= tf) & (tn >= 0.0) & (tn < best)
                if not hit.any():
                    continue
                best = np.where(hit, tn, best)
                # entry face: the axis with the largest near distance
                axis_x = (txn >= tyn) & (txn >= tzn)
                axis_y = ~axis_x & (tyn >= tzn)
                cosv = np.where(axis_x, np.abs(ldx), np.where(axis_y, np.abs(ldy), np.abs(dz)))
                best_cos = np.where(hit, cosv, best_cos)
                best_idx = np.where(hit, k, best_idx)
        ok = (best >= min_range) & (best <= max_range)
        return np.where(ok, best, np.inf), np.where(ok, best_cos, 0.0), np.where(ok, best_idx, -1)


class MujocoCaster:
    """Reference caster: `mujoco.mj_ray` per point (slow in Python, exact)."""

    def __init__(self, scene: Scene) -> None:
        import mujoco

        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(scene.to_mjcf())
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.sensor_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "sensor")
        self._geomid = np.zeros(1, dtype=np.int32)
        # geom name "<kind>_<i>" -> box index i (the sensor's own geoms are excluded from the casts)
        self._box_of_geom = np.full(self.model.ngeom, -1, dtype=np.int64)
        for g in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            tail = name.rsplit("_", 1)[-1]
            if name.split("_", 1)[0] in ("wall", "obstacle", "floor", "ceiling") and tail.isdigit():
                self._box_of_geom[g] = int(tail)
        self.n_surfaces = len(scene.boxes)
        self._boxes = [(np.array(b.center), np.array(b.half), math.cos(b.yaw), math.sin(b.yaw)) for b in scene.boxes]

    def cast(self, origins, dirs, min_range=0.1, max_range=40.0):
        mj = self._mj
        n = origins.shape[0]
        out = np.full(n, np.inf)
        cos = np.zeros(n)
        idx = np.full(n, -1, dtype=np.int64)
        for i in range(n):
            d = mj.mj_ray(self.model, self.data, origins[i], dirs[i], None, 1, self.sensor_body, self._geomid)
            if d >= 0.0 and min_range <= d <= max_range:
                k = int(self._box_of_geom[self._geomid[0]])
                out[i], idx[i] = d, k
                if k >= 0:
                    cos[i] = _face_cos(*self._boxes[k], origins[i] + d * dirs[i], dirs[i])
        return out, cos, idx
