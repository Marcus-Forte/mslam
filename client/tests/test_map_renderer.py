import contextlib
import unittest
from typing import Any

import numpy as np

from tests.helpers import make_chunk
from map_renderer import MapRenderer
from map_store import MapStore


class FakeNode:
    def __init__(self, scene: "FakeScene", name: str, points: np.ndarray, colors: np.ndarray) -> None:
        self._scene = scene
        self.name = name
        self._points = points
        self._colors = colors
        self.sent_points = len(points)  # points sent to the browser so far for this node, in total
        self.removed = False

    @property
    def points(self) -> np.ndarray:
        return self._points

    @points.setter
    def points(self, value: np.ndarray) -> None:
        self._points = value
        self.sent_points += len(value)
        self._scene.events.append(("points", self.name, len(value)))
        self._scene.points_in_atomic.append(self._scene.in_atomic)

    @property
    def colors(self) -> np.ndarray:
        return self._colors

    @colors.setter
    def colors(self, value: np.ndarray) -> None:
        self._colors = value
        self._scene.events.append(("colors", self.name, len(value)))

    def remove(self) -> None:
        self.removed = True
        self._scene.nodes.pop(self.name)
        self._scene.events.append(("remove", self.name, 0))


class FakeScene:
    def __init__(self) -> None:
        self.nodes: dict[str, FakeNode] = {}
        self.events: list[tuple[str, str, int]] = []
        self.in_atomic = False
        self.points_in_atomic: list[bool] = []

    def add_point_cloud(self, name: str, points: np.ndarray, colors: np.ndarray, **_: Any) -> FakeNode:
        assert len(points) == len(colors)
        node = FakeNode(self, name, points, colors)
        self.nodes[name] = node
        self.events.append(("add", name, len(points)))
        return node

    @contextlib.contextmanager
    def atomic(self):
        self.in_atomic = True
        try:
            yield
        finally:
            self.in_atomic = False


class RendererTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = MapStore(capacity=100)
        self.scene = FakeScene()
        self.renderer = MapRenderer(self.store, self.scene, point_size=0.1, atomic=self.scene.atomic)

    def apply(self, *args: Any, **kwargs: Any) -> None:
        self.store.apply(make_chunk(*args, **kwargs))
        self.renderer.notify()

    def test_nothing_is_sent_without_a_change(self) -> None:
        self.assertFalse(self.renderer.flush())
        self.assertEqual(self.scene.events, [])

    def test_many_chunks_between_flushes_cause_one_update(self) -> None:
        self.apply(7, 0, 0)
        self.apply(7, 0, 1, count=10)
        self.assertTrue(self.renderer.flush())
        self.scene.events.clear()
        for k in range(20):
            self.apply(7, 1 + k, 2 + k, count=3, start=10 + 3 * k)
        self.assertTrue(self.renderer.flush())
        self.assertFalse(self.renderer.flush())
        self.assertEqual([e[0] for e in self.scene.events], ["points", "colors"])
        self.assertEqual(self.scene.nodes["/slam/map/open"].points.shape, (70, 3))

    def test_points_and_colors_are_updated_together_inside_an_atomic_window(self) -> None:
        self.apply(7, 0, 1, count=10)
        self.renderer.flush()
        node = self.scene.nodes["/slam/map/open"]
        self.apply(7, 1, 2, count=5, start=10)
        self.renderer.flush()
        self.assertEqual(self.scene.points_in_atomic, [True])
        self.assertEqual(len(node.points), len(node.colors))

    def test_sealed_blocks_become_nodes_once_and_are_never_resent(self) -> None:
        self.apply(7, 0, 1, count=250)
        self.renderer.flush()
        self.assertEqual(sorted(self.scene.nodes), ["/slam/map/block_0", "/slam/map/block_1", "/slam/map/open"])
        sent = {name: node.sent_points for name, node in self.scene.nodes.items()}

        for k in range(40):
            self.apply(7, 1 + k, 2 + k, count=10, start=250 + 10 * k)
            self.renderer.flush()
        self.assertEqual(len(self.scene.nodes), 7)  # 650 points: 6 sealed + open
        for name in ("/slam/map/block_0", "/slam/map/block_1"):
            self.assertEqual(self.scene.nodes[name].sent_points, sent[name])
        adds = [e for e in self.scene.events if e[0] == "add" and e[1].startswith("/slam/map/block")]
        self.assertEqual(len(adds), 6)  # each sealed block was created exactly once
        # Data sent per update is bounded by the capacity however large the map is.
        self.assertTrue(all(e[2] <= self.store.capacity for e in self.scene.events))

    def test_epoch_change_removes_all_map_nodes(self) -> None:
        self.apply(7, 0, 1, count=250)
        self.renderer.flush()
        self.apply(8, 0, 0)
        self.renderer.flush()
        self.assertEqual(self.scene.nodes, {})
        self.apply(8, 0, 1, count=30)
        self.renderer.flush()
        self.assertEqual(sorted(self.scene.nodes), ["/slam/map/open"])
        self.assertEqual(self.scene.nodes["/slam/map/open"].points.shape, (30, 3))

    def test_clear_and_refill_between_two_flushes_is_rebuilt(self) -> None:
        self.apply(7, 0, 1, count=250)
        self.renderer.flush()
        self.apply(8, 0, 0)
        self.apply(8, 0, 1, count=120, start=1000)
        self.renderer.flush()
        self.assertEqual(sorted(self.scene.nodes), ["/slam/map/block_0", "/slam/map/open"])
        self.assertEqual(self.scene.nodes["/slam/map/block_0"].points[0, 0], 1000.0)


if __name__ == "__main__":
    unittest.main()
