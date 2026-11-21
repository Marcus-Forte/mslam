import contextlib
import unittest
from typing import Any

import numpy as np

from keyframe_renderer import KeyframeRenderer
from keyframe_store import KeyframeStore
from tests.helpers import make_added, make_epoch, make_pose, make_poses


class FakeNode:
    def __init__(self, scene: "FakeScene", name: str, points: np.ndarray, position: tuple, wxyz: tuple) -> None:
        self._scene = scene
        self.name = name
        self.points = points
        self.sent_points = len(points)  # points sent for this node: set once, at creation
        self._position = position
        self._wxyz = wxyz

    @property
    def position(self) -> tuple:
        return self._position

    @position.setter
    def position(self, value: tuple) -> None:
        self._position = value
        self._scene.events.append(("position", self.name, self._scene.in_atomic))

    @property
    def wxyz(self) -> tuple:
        return self._wxyz

    @wxyz.setter
    def wxyz(self, value: tuple) -> None:
        self._wxyz = value
        self._scene.events.append(("wxyz", self.name, self._scene.in_atomic))

    def remove(self) -> None:
        self._scene.nodes.pop(self.name)
        self._scene.events.append(("remove", self.name, False))


class FakeFrame:
    def __init__(
        self, scene: "FakeScene", name: str, position: tuple, wxyz: tuple, visible: bool, kwargs: dict
    ) -> None:
        self._scene = scene
        self.name = name
        self.position = position
        self.wxyz = wxyz
        self.kwargs = kwargs
        self._visible = visible
        self.removed = False

    @property
    def visible(self) -> bool:
        return self._visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self._visible = value
        self._scene.events.append(("visible", self.name, value))

    def remove(self) -> None:
        self.removed = True
        self._scene.frames.pop(self.name)
        self._scene.events.append(("remove", self.name, False))


class FakeScene:
    def __init__(self) -> None:
        self.nodes: dict[str, FakeNode] = {}
        self.frames: dict[str, FakeFrame] = {}
        self.events: list[tuple[str, str, Any]] = []
        self.in_atomic = False

    def add_point_cloud(self, name: str, points: np.ndarray, colors: np.ndarray, **kwargs: Any) -> FakeNode:
        assert len(points) == len(colors)
        node = FakeNode(self, name, points, kwargs["position"], kwargs["wxyz"])
        self.nodes[name] = node
        self.events.append(("add", name, len(points)))
        return node

    def add_frame(self, name: str, *, position: tuple, wxyz: tuple, visible: bool = True, **kwargs: Any) -> FakeFrame:
        frame = FakeFrame(self, name, position, wxyz, visible, kwargs)
        self.frames[name] = frame
        self.events.append(("add_frame", name, visible))
        return frame

    @contextlib.contextmanager
    def atomic(self):
        self.in_atomic = True
        try:
            yield
        finally:
            self.in_atomic = False


class RendererTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = KeyframeStore()
        self.scene = FakeScene()
        self.renderer = KeyframeRenderer(self.store, self.scene, point_size=0.1, atomic=self.scene.atomic)

    def apply(self, message: Any) -> None:
        self.store.apply(message)
        self.renderer.notify()

    def test_nothing_is_sent_without_a_change(self) -> None:
        self.assertFalse(self.renderer.flush())
        self.assertEqual(self.scene.events, [])

    def test_each_keyframe_becomes_one_node_placed_by_its_pose(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0, count=6))
        self.apply(make_added(7, 1, count=4, pose=make_pose(x=2.0, yaw=np.pi / 2)))
        self.assertTrue(self.renderer.flush())

        self.assertEqual(sorted(self.scene.nodes), ["/slam/keyframes/kf_0", "/slam/keyframes/kf_1"])
        node = self.scene.nodes["/slam/keyframes/kf_1"]
        self.assertEqual(len(node.points), 4)
        np.testing.assert_allclose(node.position, [2.0, 0.0, 0.0])
        np.testing.assert_allclose(node.wxyz, [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])

    def test_a_pose_update_only_moves_nodes_and_never_resends_points(self) -> None:
        self.apply(make_epoch(7))
        for keyframe_id in range(4):
            self.apply(make_added(7, keyframe_id, count=50))
        self.renderer.flush()
        self.scene.events.clear()
        sent = {name: node.sent_points for name, node in self.scene.nodes.items()}

        self.apply(make_poses(7, 3, [make_pose(), make_pose(x=1.01), make_pose(x=2.0), make_pose(x=3.02, yaw=0.01)]))
        self.assertTrue(self.renderer.flush())

        kinds = {event[0] for event in self.scene.events}
        self.assertEqual(kinds, {"position", "wxyz"})  # no add, no points
        # Only the keyframes that moved, all inside one atomic window.
        self.assertEqual({e[1] for e in self.scene.events}, {"/slam/keyframes/kf_1", "/slam/keyframes/kf_3"})
        self.assertTrue(all(event[2] for event in self.scene.events))
        self.assertEqual(self.renderer.pose_updates, 2)
        self.assertEqual({n: node.sent_points for n, node in self.scene.nodes.items()}, sent)
        np.testing.assert_allclose(self.scene.nodes["/slam/keyframes/kf_1"].position, [1.01, 0.0, 0.0])

    def test_each_keyframe_gets_a_small_axis_at_its_pose(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0))
        self.apply(make_added(7, 1, pose=make_pose(x=2.0, yaw=np.pi / 2)))
        self.renderer.flush()

        self.assertEqual(sorted(self.scene.frames), ["/slam/keyframes/axes_0", "/slam/keyframes/axes_1"])
        frame = self.scene.frames["/slam/keyframes/axes_1"]
        np.testing.assert_allclose(frame.position, [2.0, 0.0, 0.0])
        np.testing.assert_allclose(frame.wxyz, [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
        # A thin triad of about 0.2 m, and visible (the checkbox starts on).
        self.assertTrue(frame.kwargs["show_axes"])
        self.assertTrue(0.1 <= frame.kwargs["axes_length"] <= 0.25)
        self.assertLess(frame.kwargs["axes_radius"], 0.02)
        self.assertTrue(frame.visible)

    def test_a_pose_update_moves_only_the_frames_that_changed_in_the_same_window(self) -> None:
        self.apply(make_epoch(7))
        for keyframe_id in range(3):
            self.apply(make_added(7, keyframe_id))
        self.renderer.flush()
        self.scene.events.clear()

        self.apply(make_poses(7, 2, [make_pose(), make_pose(x=1.0), make_pose(x=2.5, yaw=0.01)]))
        self.renderer.flush()

        np.testing.assert_allclose(self.scene.frames["/slam/keyframes/axes_2"].position, [2.5, 0.0, 0.0])
        np.testing.assert_allclose(self.scene.frames["/slam/keyframes/axes_1"].position, [1.0, 0.0, 0.0])
        # No frame was re-created, and the cloud and axes of keyframe 2 moved together.
        self.assertFalse([e for e in self.scene.events if e[0] == "add_frame"])
        np.testing.assert_allclose(
            self.scene.frames["/slam/keyframes/axes_2"].wxyz, self.scene.nodes["/slam/keyframes/kf_2"].wxyz
        )
        self.assertEqual(self.renderer.pose_updates, 1)  # keyframe 1 had not moved
        np.testing.assert_allclose(self.scene.frames["/slam/keyframes/axes_0"].position, [0.0, 0.0, 0.0])

    def test_an_epoch_change_removes_the_axes_with_the_clouds(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0))
        self.apply(make_added(7, 1))
        self.renderer.flush()
        self.apply(make_epoch(8))
        self.renderer.flush()
        self.assertEqual(self.scene.frames, {})
        self.assertEqual(self.scene.nodes, {})
        self.apply(make_added(8, 0))
        self.renderer.flush()
        self.assertEqual(sorted(self.scene.frames), ["/slam/keyframes/axes_0"])

    def test_the_toggle_hides_and_shows_the_axes_without_re_adding_them(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0))
        self.apply(make_added(7, 1))
        self.renderer.flush()
        self.scene.events.clear()

        self.renderer.set_axes_visible(False)
        self.assertTrue(self.renderer.flush())
        self.assertFalse(any(f.visible for f in self.scene.frames.values()))
        self.assertTrue(all(node.sent_points > 0 for node in self.scene.nodes.values()))  # clouds untouched

        # A keyframe that arrives while hidden starts hidden.
        self.apply(make_added(7, 2))
        self.renderer.flush()
        self.assertFalse(self.scene.frames["/slam/keyframes/axes_2"].visible)

        self.renderer.set_axes_visible(True)
        self.renderer.flush()
        self.assertTrue(all(f.visible for f in self.scene.frames.values()))
        added = [e for e in self.scene.events if e[0] == "add_frame"]
        self.assertEqual([e[1] for e in added], ["/slam/keyframes/axes_2"])  # nothing re-created

    def test_new_keyframes_are_added_once_between_flushes(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0))
        self.renderer.flush()
        self.apply(make_added(7, 1))
        self.apply(make_added(7, 2))
        self.renderer.flush()
        self.renderer.flush()
        adds = [event for event in self.scene.events if event[0] == "add"]
        self.assertEqual([event[1] for event in adds], [f"/slam/keyframes/kf_{k}" for k in range(3)])

    def test_epoch_change_removes_every_node(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0))
        self.apply(make_added(7, 1))
        self.renderer.flush()
        self.apply(make_epoch(8))
        self.renderer.flush()
        self.assertEqual(self.scene.nodes, {})
        self.apply(make_added(8, 0, count=3))
        self.renderer.flush()
        self.assertEqual(sorted(self.scene.nodes), ["/slam/keyframes/kf_0"])

    def test_clear_and_refill_between_two_flushes_is_rebuilt(self) -> None:
        self.apply(make_epoch(7))
        self.apply(make_added(7, 0, count=5))
        self.renderer.flush()
        self.apply(make_epoch(8))
        self.apply(make_added(8, 0, count=9))
        self.renderer.flush()
        self.assertEqual(len(self.scene.nodes["/slam/keyframes/kf_0"].points), 9)


if __name__ == "__main__":
    unittest.main()
