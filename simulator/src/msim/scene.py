"""Floor-plan scenes: YAML -> oriented boxes -> MJCF.

A scene is a set of oriented boxes (walls, door lintels, obstacles, floor,
ceiling). The same list feeds the numpy ray caster (`raycast.BoxCaster`) and the
MJCF written for MuJoCo (the viewer and the cross-check caster), so the two
cannot disagree about the geometry.

YAML format (lengths in metres, angles in degrees, z up):

    name: two_rooms
    wall_height: 2.8          # default for all walls
    wall_thickness: 0.2       # default; a wall's centre line is the segment
    floor: true
    ceiling: true
    start: {position: [x, y, z], yaw_deg: 0}
    walls:                    # free segments
      - from: [0, 0]
        to: [6, 0]
        thickness: 0.2        # optional overrides: thickness, height
        openings:             # doors/windows: centre distance `at` from `from`
          - {at: 2.0, width: 1.0, height: 2.1, sill: 0.0}
    rooms:                    # closed polygons; every edge is a wall
      - name: a
        polygon: [[0, 0], [6, 0], [6, 5], [0, 5]]
        openings: [{edge: 1, at: 2.5, width: 1.0, height: 2.1}]   # edge i = pts[i] -> pts[i+1]
    boxes:                    # free-standing obstacles
      - {name: crate, center: [2, 2], size: [0.6, 0.6, 0.8], yaw_deg: 30, z: 0.0}

Surface reflectivity (diffuse reflectance at the Mid360's 905 nm, 0..1; see `reflectivity.py`):

    reflectivity: {wall: 0.4, floor: 0.2, ceiling: 0.45, obstacle: 0.3}   # per-kind defaults (these values)
    walls:   [{from: .., to: .., reflectivity: 0.1}]                      # per wall / room / box override
    boxes:   [{name: sign, ..., retroreflective: true}]                    # retro-reflector: intensity 151..255
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from xml.sax.saxutils import quoteattr

import numpy as np
import yaml

SLAB = 0.1  # thickness of the floor and ceiling slabs


@dataclass(frozen=True)
class Box:
    """Oriented box: yaw about z only, `half` = half extents along its own axes."""

    name: str
    center: tuple[float, float, float]
    half: tuple[float, float, float]
    yaw: float = 0.0  # rad
    kind: str = "wall"  # wall | obstacle | floor | ceiling
    reflectivity: float = 0.4  # diffuse reflectance at 905 nm, 0..1
    retro: bool = False        # retro-reflector (Livox reports 151..255)


KINDS = ("wall", "obstacle", "floor", "ceiling")
# Defaults per kind, calibrated so that the `two_rooms_loop` run has about the median intensity of the real
# `living_room_garden` recording (48); real indoor surfaces read darker than their nominal reflectance would
# suggest (a white wall at 80 % would read 120 head-on, the recording's 99th percentile is 99).
DEFAULT_REFLECTIVITY = {"wall": 0.4, "floor": 0.2, "ceiling": 0.45, "obstacle": 0.3}


@dataclass
class Scene:
    name: str
    boxes: list[Box] = field(default_factory=list)
    wall_height: float = 2.8
    start_position: tuple[float, float, float] = (0.0, 0.0, 1.0)
    start_yaw: float = 0.0  # rad

    # ---- geometry queries -------------------------------------------------
    def solids(self) -> list[Box]:
        """Boxes the sensor body collides with (not floor and ceiling)."""
        return [b for b in self.boxes if b.kind in ("wall", "obstacle")]

    def collides(self, p, radius: float) -> bool:
        """True if a sphere of `radius` at `p` touches a wall/obstacle or leaves the floor/ceiling."""
        if p[2] < radius or p[2] > self.wall_height - radius:
            return True
        for b in self.solids():
            c, s = math.cos(-b.yaw), math.sin(-b.yaw)
            dx, dy, dz = p[0] - b.center[0], p[1] - b.center[1], p[2] - b.center[2]
            lx, ly, lz = c * dx - s * dy, s * dx + c * dy, dz
            qx = max(abs(lx) - b.half[0], 0.0)
            qy = max(abs(ly) - b.half[1], 0.0)
            qz = max(abs(lz) - b.half[2], 0.0)
            if qx * qx + qy * qy + qz * qz < radius * radius:
                return True
        return False

    def bounds_xy(self) -> tuple[np.ndarray, np.ndarray]:
        pts = []
        for b in self.boxes:
            if b.kind in ("floor", "ceiling"):
                continue
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    pts.append((b.center[0] + c * sx * b.half[0] - s * sy * b.half[1],
                                b.center[1] + s * sx * b.half[0] + c * sy * b.half[1]))
        a = np.array(pts)
        return a.min(0), a.max(0)

    # ---- MJCF ---------------------------------------------------------------
    def to_mjcf(self) -> str:
        """MJCF with one box geom per Box plus a mocap body `sensor` (no physics is run)."""
        colors = {"wall": "0.75 0.72 0.65 1", "obstacle": "0.55 0.40 0.30 1",
                  "floor": "0.35 0.38 0.40 1", "ceiling": "0.8 0.85 0.9 0.12"}
        lines = [
            '<mujoco model=%s>' % quoteattr(self.name),
            '  <compiler angle="radian"/>',
            '  <visual><global offwidth="1280" offheight="720"/></visual>',
            '  <worldbody>',
            '    <light pos="0 0 6" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>',
        ]
        for i, b in enumerate(self.boxes):
            lines.append(
                '    <geom name=%s type="box" pos="%.6f %.6f %.6f" size="%.6f %.6f %.6f" '
                'euler="0 0 %.9f" rgba="%s" contype="0" conaffinity="0"/>'
                % (quoteattr(f"{b.kind}_{i}"), *b.center, *b.half, b.yaw, colors[b.kind]))
        x, y, z = self.start_position
        lines += [
            '    <body name="sensor" mocap="true" pos="%.6f %.6f %.6f">' % (x, y, z),
            '      <geom name="sensor_body" type="box" size="0.06 0.06 0.04" rgba="0.9 0.2 0.1 1" contype="0" conaffinity="0"/>',
            '      <geom name="sensor_nose" type="box" pos="0.09 0 0" size="0.04 0.02 0.02" rgba="1 1 0.1 1" contype="0" conaffinity="0"/>',
            '    </body>',
            '  </worldbody>',
            '</mujoco>',
        ]
        return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# YAML -> Scene
# ---------------------------------------------------------------------------

def _num(d: dict[str, Any], key: str, default: float | None = None) -> float:
    v = d.get(key, default)
    if v is None:
        raise ValueError(f"missing key '{key}' in {d}")
    return float(v)


def _reflectivity(v: Any, where: str) -> float:
    r = float(v)
    if not 0.0 <= r <= 1.0:
        raise ValueError(f"{where}: reflectivity must be in [0, 1] (diffuse reflectance), got {v}")
    return r


def _wall_boxes(p0, p1, thickness: float, height: float, openings: list[dict], prefix: str,
                reflectivity: float = DEFAULT_REFLECTIVITY["wall"], retro: bool = False) -> list[Box]:
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    seg = p1 - p0
    length = float(np.hypot(*seg))
    if length <= 0.0:
        raise ValueError(f"zero-length wall {p0} -> {p1}")
    if thickness <= 0.0 or height <= 0.0:
        raise ValueError("wall thickness and height must be positive")
    u = seg / length
    yaw = math.atan2(seg[1], seg[0])
    ops = []
    for o in openings:
        at, w = _num(o, "at"), _num(o, "width")
        top, sill = _num(o, "height", 2.1), _num(o, "sill", 0.0)
        a, b = at - w / 2.0, at + w / 2.0
        if w <= 0 or a < 0 or b > length or not (0.0 <= sill < top < height + 1e-9):
            raise ValueError(f"opening {o} does not fit a wall of length {length:.3f} and height {height:.3f}")
        ops.append((a, b, sill, min(top, height)))
    ops.sort()
    for (_, b0, _, _), (a1, _, _, _) in zip(ops, ops[1:]):
        if a1 < b0:
            raise ValueError("overlapping openings")

    # pieces: (s0, s1, z0, z1); the wall's two outer ends grow by t/2 so corners close
    pieces: list[tuple[float, float, float, float]] = []
    cursor = 0.0
    for a, b, sill, top in ops:
        if a > cursor:
            pieces.append((cursor, a, 0.0, height))
        if sill > 0.0:
            pieces.append((a, b, 0.0, sill))
        if top < height:
            pieces.append((a, b, top, height))
        cursor = b
    if cursor < length:
        pieces.append((cursor, length, 0.0, height))

    boxes = []
    for k, (s0, s1, z0, z1) in enumerate(pieces):
        e0 = s0 - (thickness / 2.0 if s0 == 0.0 else 0.0)
        e1 = s1 + (thickness / 2.0 if s1 == length else 0.0)
        mid = p0 + u * (e0 + e1) / 2.0
        boxes.append(Box(f"{prefix}_{k}", (float(mid[0]), float(mid[1]), (z0 + z1) / 2.0),
                         ((e1 - e0) / 2.0, thickness / 2.0, (z1 - z0) / 2.0), yaw, "wall", reflectivity, retro))
    return boxes


def scene_from_dict(d: dict[str, Any]) -> Scene:
    name = str(d.get("name", "scene"))
    height = float(d.get("wall_height", 2.8))
    thick = float(d.get("wall_thickness", 0.2))
    boxes: list[Box] = []
    refl_cfg = d.get("reflectivity", {}) or {}
    if not isinstance(refl_cfg, dict) or set(refl_cfg) - set(KINDS):
        raise ValueError(f"'reflectivity' must map some of {KINDS} to a value in [0, 1], got {refl_cfg}")
    refl = {k: _reflectivity(refl_cfg.get(k, v), f"reflectivity.{k}") for k, v in DEFAULT_REFLECTIVITY.items()}

    def surface(item: dict, kind: str, where: str) -> tuple[float, bool]:
        return (_reflectivity(item.get("reflectivity", refl[kind]), where),
                bool(item.get("retroreflective", False)))

    for i, w in enumerate(d.get("walls", []) or []):
        boxes += _wall_boxes(w["from"], w["to"], float(w.get("thickness", thick)),
                             float(w.get("height", height)), w.get("openings", []) or [], f"wall{i}",
                             *surface(w, "wall", f"walls[{i}]"))
    for r_i, r in enumerate(d.get("rooms", []) or []):
        poly = r["polygon"]
        if len(poly) < 3:
            raise ValueError("a room polygon needs at least 3 points")
        n = len(poly)
        per_edge: dict[int, list[dict]] = {}
        for o in r.get("openings", []) or []:
            e = int(o["edge"])
            if not 0 <= e < n:
                raise ValueError(f"opening edge {e} out of range for room {r.get('name', r_i)}")
            per_edge.setdefault(e, []).append(o)
        rname = r.get("name", f"room{r_i}")
        r_refl, r_retro = surface(r, "wall", f"rooms[{r_i}]")
        for e in range(n):
            boxes += _wall_boxes(poly[e], poly[(e + 1) % n], float(r.get("thickness", thick)),
                                 float(r.get("height", height)), per_edge.get(e, []), f"{rname}_e{e}",
                                 r_refl, r_retro)
    for i, b in enumerate(d.get("boxes", []) or []):
        sx, sy, sz = (float(v) for v in b["size"])
        if min(sx, sy, sz) <= 0:
            raise ValueError("box sizes must be positive")
        z0 = float(b.get("z", 0.0))
        boxes.append(Box(str(b.get("name", f"box{i}")),
                         (float(b["center"][0]), float(b["center"][1]), z0 + sz / 2.0),
                         (sx / 2.0, sy / 2.0, sz / 2.0), math.radians(float(b.get("yaw_deg", 0.0))), "obstacle",
                         *surface(b, "obstacle", f"boxes[{i}]")))
    if not boxes:
        raise ValueError("scene has no walls, rooms or boxes")

    scene = Scene(name=name, boxes=boxes, wall_height=height)
    lo, hi = scene.bounds_xy()
    margin = float(d.get("margin", 1.0))
    c = (lo + hi) / 2.0
    half = (hi - lo) / 2.0 + margin
    if d.get("floor", True):
        scene.boxes.append(Box("floor", (c[0], c[1], -SLAB / 2.0), (half[0], half[1], SLAB / 2.0), 0.0, "floor",
                               refl["floor"]))
    if d.get("ceiling", True):
        scene.boxes.append(Box("ceiling", (c[0], c[1], height + SLAB / 2.0), (half[0], half[1], SLAB / 2.0), 0.0,
                               "ceiling", refl["ceiling"]))
    st = d.get("start", {}) or {}
    pos = st.get("position", [float(c[0]), float(c[1]), 1.0])
    scene.start_position = (float(pos[0]), float(pos[1]), float(pos[2]))
    scene.start_yaw = math.radians(float(st.get("yaw_deg", 0.0)))
    return scene


def load_scene(path: str | Path) -> Scene:
    with open(path) as f:
        d = yaml.safe_load(f)
    if not isinstance(d, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return scene_from_dict(d)


def write_mjcf(scene: Scene, path: str | Path) -> None:
    Path(path).write_text(scene.to_mjcf())
