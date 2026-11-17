"""MuJoCo passive viewer with keyboard flight control.

On macOS the passive viewer must run under `mjpython` (`uv run mjpython -m msim serve ...`),
because the GUI has to own the main thread. The simulation engine runs in a
background thread either way.

MuJoCo's simulate window forwards key presses and OS key repeats to the callback,
but not releases. A key therefore counts as held for a short window after each
event (Tab / Shift+Tab are the viewer's own panel toggles, reserved): 0.45 s after a first press (covers the OS repeat delay), 0.18 s after a
repeat. Tapping nudges, holding flies, and the sensor stops about 0.2 s after
you let go (the jerk-limited controller then eases it to rest).
"""

from __future__ import annotations

import threading
import time

import numpy as np

from .engine import GroundTruth, SimEngine
from .motion import Command, GameController
from .scene import Scene
from .sensors import ScanData

# GLFW key codes.
#
# MuJoCo's viewer keeps its own hotkeys while our callback runs (every letter toggles a display flag:
# W wireframe, S shadows, R reflections, D static bodies, ...). The flags in `mujoco.mjVISSTRING` live in
# `handle.opt` and are restored every frame (see `run_viewer`), but the ones in `mujoco.mjRNDSTRING`
# (S W R L K G, '/' and ',') live in the viewer's private scene and cannot be undone from Python, so the
# controls avoid them. Letters left over are used only when their built-in effect is invisible in a scene.
K_Z, K_X, K_Q, K_E, K_Y, K_H = 90, 88, 81, 69, 89, 72
# Roll U/O, translation-mode toggle B, level-hold N: their built-in toggles (Actuator, Perturb Object,
# Perturb Force, Island) are all mjVISSTRING flags, restored every frame, and invisible in these scenes.
K_U, K_O, K_B, K_N = 85, 79, 66, 78
K_TAB = 258   # reserved: the viewer shows/hides its left panel (Shift+Tab: right panel)
K_ENTER, K_SPACE = 257, 32
K_RIGHT, K_LEFT, K_DOWN, K_UP = 262, 263, 264, 265
CONTROL_KEYS = {K_Z, K_X, K_Q, K_E, K_Y, K_H, K_U, K_O, K_N, K_RIGHT, K_LEFT, K_UP, K_DOWN}
RESERVED_KEYS = {K_TAB}

HELP = ("Up/Down arrows forward/back, Left/Right arrows turn, Z/X strafe left/right, E/Q up/down, "
        "Y/H look up/down, U/O roll left/right, N hold to level, B toggle level/body translation, "
        "Space brake, Enter reset to the start pose and attitude, Tab / Shift+Tab show/hide the viewer panels")


class KeyboardControl:
    FIRST_HOLD, REPEAT_HOLD, REPEAT_WINDOW = 0.45, 0.18, 0.6

    def __init__(self, controller: GameController) -> None:
        self.controller = controller
        self._until: dict[int, float] = {}
        self._last: dict[int, float] = {}
        self._lock = threading.Lock()

    def on_key(self, keycode: int) -> None:
        now = time.perf_counter()
        with self._lock:
            if keycode == K_ENTER:
                self.controller.reset()
                self._until.clear()
            elif keycode == K_B:
                print(f"translation mode: {self.controller.toggle_frame_mode()}")
            elif keycode == K_SPACE:
                self._until.clear()
            elif keycode in CONTROL_KEYS:
                repeat = now - self._last.get(keycode, -1e9) < self.REPEAT_WINDOW
                self._until[keycode] = now + (self.REPEAT_HOLD if repeat else self.FIRST_HOLD)
                self._last[keycode] = now
            self._publish(now)

    def tick(self) -> None:
        """Call at UI rate so expired keys release."""
        with self._lock:
            self._publish(time.perf_counter())

    def _publish(self, now: float) -> None:
        def held(k: int) -> float:
            return 1.0 if self._until.get(k, 0.0) > now else 0.0
        self.controller.command = Command(
            forward=held(K_UP) - held(K_DOWN), left=held(K_Z) - held(K_X), up=held(K_E) - held(K_Q),
            yaw_rate=held(K_LEFT) - held(K_RIGHT), pitch_rate=held(K_Y) - held(K_H),
            roll_rate=held(K_O) - held(K_U), level=bool(held(K_N)))


class ScanPoints:
    """Keeps the last scan's hits in the world frame (pose at the scan end; deskew ignored) for drawing."""

    def __init__(self, max_points: int = 400) -> None:
        self.max_points = max_points
        self.world: np.ndarray | None = None
        self.version = 0

    def __call__(self, scan: ScanData, gt: GroundTruth) -> None:
        pts = scan.xyz[scan.hit]
        if len(pts) > self.max_points:
            pts = pts[:: len(pts) // self.max_points + 1]
        self.world = gt.p + pts.astype(np.float64) @ gt.R.T
        self.version += 1


def launch_kwargs(show_ui: bool) -> dict:
    """Keyword arguments for `mujoco.viewer.launch_passive` (the side panels are hidden unless `show_ui`)."""
    return {"show_left_ui": show_ui, "show_right_ui": show_ui}


def run_viewer(scene: Scene, engine: SimEngine, keyboard: KeyboardControl | None, points: ScanPoints | None,
               show_ui: bool = False) -> None:
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_string(scene.to_mjcf())
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "sensor")
    mocap = model.body_mocapid[body]
    print(HELP if keyboard else "Scripted trajectory: watching only.")
    identity = np.eye(3).flatten()
    drawn = -1
    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard.on_key if keyboard else None,
                                      **launch_kwargs(show_ui)) as handle:
        default_flags = np.array(handle.opt.flags, copy=True)
        while handle.is_running() and not engine.done.is_set():
            handle.opt.flags[:] = default_flags   # undo the viewer's own letter hotkeys (see top of file)
            k = engine.latest
            if k is not None:
                from .geometry import quat_from_rot
                data.mocap_pos[mocap] = k.p
                data.mocap_quat[mocap] = quat_from_rot(k.R)
            if keyboard:
                keyboard.tick()
            with handle.lock():
                mujoco.mj_forward(model, data)
                w = points.world if points else None
                if w is not None and points.version != drawn:  # redraw only per new scan (GIL)
                    drawn = points.version
                    scn = handle.user_scn
                    n = min(len(w), scn.maxgeom)
                    for i in range(n):
                        mujoco.mjv_initGeom(scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]),
                                            w[i], identity, np.array([0.1, 0.9, 0.2, 1.0], dtype=np.float32))
                    scn.ngeom = n
            handle.sync()
            time.sleep(1.0 / 30.0)
    engine.stop()
