"""Renders a `KeyframeStore` into a viser-like scene: one point-cloud node per keyframe.

A keyframe's points are sent once, when its node is created, in the keyframe's own frame; the node's `position` and
`wxyz` place it in the map. When the server's optimization moves keyframes, only the poses of those nodes change:
no point is re-sent. The renderer works from store snapshots (state, not deltas), at a bounded rate, like
`map_renderer.MapRenderer`.

Each keyframe also gets a small coordinate axis triad (`scene.add_frame`), a sibling node placed by the same pose, so
the poses of the graph are visible even where a keyframe owns few points. The axes follow the clouds in the same
atomic window and can be hidden without being re-created (`set_axes_visible`).

Nothing here imports viser: `scene` only needs `add_point_cloud(name, points, colors, point_size=, point_shape=,
position=, wxyz=)` and `add_frame(name, show_axes=, axes_length=, axes_radius=, position=, wxyz=, visible=)`, each
returning a handle with `position`, `wxyz` and `remove()` (a frame also `visible`).
"""

import contextlib
import logging
import threading
from collections.abc import Callable
from typing import Any

from keyframe_store import KeyframeStore
from map_renderer import FLUSH_INTERVAL_SEC

logger = logging.getLogger(__name__)

DEFAULT_NODE_PREFIX = "/slam/keyframes"

AXES_LENGTH_M = 0.18
AXES_RADIUS_M = 0.006
"""A thin triad of about 0.2 m: visible next to a room-scale map without hiding its points."""


def _no_atomic() -> contextlib.AbstractContextManager[None]:
    return contextlib.nullcontext()


class KeyframeRenderer:
    def __init__(
        self,
        store: KeyframeStore,
        scene: Any,
        point_size: float,
        point_shape: str = "rounded",
        atomic: Callable[[], contextlib.AbstractContextManager[None]] = _no_atomic,
        node_prefix: str = DEFAULT_NODE_PREFIX,
    ) -> None:
        """`atomic` returns a context in which scene updates reach a client as one unit (`ViserServer.atomic`):
        a pose update of many keyframes is applied by a browser all at once. Nodes are named
        `<node_prefix>/kf_<id>` (points) and `<node_prefix>/axes_<id>` (axes)."""
        self._store = store
        self._scene = scene
        self._point_size = point_size
        self._point_shape = point_shape
        self._atomic = atomic
        self._node_prefix = node_prefix
        self._dirty = threading.Event()
        # Only the flushing thread touches the fields below.
        self._generation: int | None = None
        self._nodes: list[Any] = []
        self._axes: list[Any] = []
        self._pose_versions: list[int] = []
        self._axes_visible = True  # wanted (set from any thread)
        self._axes_shown = True  # what the nodes currently have (flushing thread)
        self.flush_count = 0
        """Flushes that sent something."""
        self.pose_updates = 0
        """Node poses changed after creation (no points involved)."""

    def notify(self) -> None:
        """Mark the store as changed. Cheap and callable from any thread; many calls cause one flush."""
        self._dirty.set()

    def set_axes_visible(self, visible: bool) -> None:
        """Show or hide every keyframe axis (and the ones created later) without re-creating them. Callable from any
        thread; applied by the next flush."""
        self._axes_visible = visible
        self.notify()

    def flush(self) -> bool:
        """Bring the scene up to date with the store if it changed. Returns True if anything was sent."""
        if not self._dirty.is_set():
            return False
        self._dirty.clear()
        snapshot = self._store.snapshot()

        if snapshot.generation != self._generation:
            self.clear()
            self._generation = snapshot.generation

        moved = [
            (index, view)
            for index, view in enumerate(snapshot.keyframes[: len(self._nodes)])
            if view.pose_version != self._pose_versions[index]
        ]
        if moved:
            with self._atomic():
                for index, view in moved:
                    for node in (self._nodes[index], self._axes[index]):
                        node.position = tuple(view.position)
                        node.wxyz = tuple(view.wxyz)
                    self._pose_versions[index] = view.pose_version
            self.pose_updates += len(moved)

        for view in snapshot.keyframes[len(self._nodes) :]:
            self._nodes.append(
                self._scene.add_point_cloud(
                    name=f"{self._node_prefix}/kf_{view.id}",
                    points=view.points,
                    colors=view.colors,
                    point_size=self._point_size,
                    point_shape=self._point_shape,
                    position=tuple(view.position),
                    wxyz=tuple(view.wxyz),
                )
            )
            self._axes.append(
                self._scene.add_frame(
                    name=f"{self._node_prefix}/axes_{view.id}",
                    show_axes=True,
                    axes_length=AXES_LENGTH_M,
                    axes_radius=AXES_RADIUS_M,
                    position=tuple(view.position),
                    wxyz=tuple(view.wxyz),
                    visible=self._axes_shown,
                )
            )
            self._pose_versions.append(view.pose_version)

        if self._axes_visible != self._axes_shown:
            self._axes_shown = self._axes_visible
            with self._atomic():
                for axes in self._axes:
                    axes.visible = self._axes_shown

        self.flush_count += 1
        return True

    def clear(self) -> None:
        """Remove every node (the store is untouched)."""
        for node in (*self._nodes, *self._axes):
            node.remove()
        self._nodes = []
        self._axes = []
        self._pose_versions = []

    def run(self, should_stop: Callable[[], bool], wait: Callable[[float], bool]) -> None:
        """Flush every `FLUSH_INTERVAL_SEC` until `wait` returns True (e.g. `Event.wait`)."""
        while not should_stop():
            if wait(FLUSH_INTERVAL_SEC):
                return
            try:
                self.flush()
            except Exception:
                logger.exception("Keyframe render flush failed")
                self._dirty.set()
