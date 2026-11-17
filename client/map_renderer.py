"""Renders a `MapStore` into a viser-like scene, incrementally and at a bounded rate.

Each sealed block becomes its own point-cloud node, created once and never re-sent. The open block is one node
whose data is re-sent when it changed, so the data sent per update is bounded by the block capacity however large
the map is. The renderer works from store snapshots (state, not deltas), so a missed or merged notification cannot
leave the scene inconsistent.

Nothing here imports viser: `scene` only needs `add_point_cloud(name, points, colors, point_size=, point_shape=)`
returning a handle with `points`, `colors` and `remove()`.
"""

import contextlib
import logging
import threading
from collections.abc import Callable
from typing import Any

import numpy as np

from map_store import MapStore

logger = logging.getLogger(__name__)

FLUSH_INTERVAL_SEC = 0.5
"""At most two updates per second."""

DEFAULT_NODE_PREFIX = "/slam/map"


def _no_atomic() -> contextlib.AbstractContextManager[None]:
    return contextlib.nullcontext()


class MapRenderer:
    def __init__(
        self,
        store: MapStore,
        scene: Any,
        point_size: float,
        point_shape: str = "rounded",
        atomic: Callable[[], contextlib.AbstractContextManager[None]] = _no_atomic,
        node_prefix: str = DEFAULT_NODE_PREFIX,
    ) -> None:
        """`atomic` returns a context in which scene updates reach a client as one unit (`ViserServer.atomic`).
        Nodes are named `<node_prefix>/block_<i>` and `<node_prefix>/open`."""
        self._node_prefix = node_prefix
        self._store = store
        self._scene = scene
        self._point_size = point_size
        self._point_shape = point_shape
        self._atomic = atomic
        self._dirty = threading.Event()
        # Only the flushing thread touches the fields below.
        self._generation: int | None = None
        self._sealed_nodes: list[Any] = []
        self._open_node: Any = None
        self.flush_count = 0
        """Flushes that sent something."""

    def notify(self) -> None:
        """Mark the store as changed. Cheap and callable from any thread; many calls cause one flush."""
        self._dirty.set()

    def flush(self) -> bool:
        """Bring the scene up to date with the store if it changed. Returns True if anything was sent."""
        if not self._dirty.is_set():
            return False
        self._dirty.clear()
        snapshot = self._store.snapshot()

        if snapshot.generation != self._generation:
            self._remove_all()
            self._generation = snapshot.generation

        for block in snapshot.sealed[len(self._sealed_nodes) :]:
            self._sealed_nodes.append(
                self._add_node(f"{self._node_prefix}/block_{block.index}", block.points, block.colors)
            )

        self._update_open(snapshot.open_points, snapshot.open_colors)
        self.flush_count += 1
        return True

    def run(self, should_stop: Callable[[], bool], wait: Callable[[float], bool]) -> None:
        """Flush every `FLUSH_INTERVAL_SEC` until `wait` returns True (e.g. `Event.wait`)."""
        while not should_stop():
            if wait(FLUSH_INTERVAL_SEC):
                return
            try:
                self.flush()
            except Exception:
                logger.exception("Map render flush failed")
                self._dirty.set()

    def _add_node(self, name: str, points: np.ndarray, colors: np.ndarray) -> Any:
        return self._scene.add_point_cloud(
            name=name, points=points, colors=colors, point_size=self._point_size, point_shape=self._point_shape
        )

    def _remove_all(self) -> None:
        for node in self._sealed_nodes:
            node.remove()
        self._sealed_nodes = []
        if self._open_node is not None:
            self._open_node.remove()
            self._open_node = None

    def _update_open(self, points: np.ndarray, colors: np.ndarray) -> None:
        if self._open_node is None:
            if len(points) > 0:
                self._open_node = self._add_node(f"{self._node_prefix}/open", points, colors)
            return
        # Points and colors are separate property updates in viser (no single-message update, still so in 1.1.1), so group them
        # in one atomic window: a browser applies both or neither, never arrays of different lengths.
        with self._atomic():
            self._open_node.points = points
            self._open_node.colors = colors
