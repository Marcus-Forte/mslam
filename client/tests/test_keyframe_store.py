import unittest

import numpy as np

from keyframe_store import KeyframeResume, KeyframeStore
from tests.helpers import make_added, make_epoch, make_pose, make_poses


class ApplyTest(unittest.TestCase):
    def test_keyframes_arrive_in_order_and_advance_the_resume_position(self) -> None:
        store = KeyframeStore()
        self.assertEqual(store.resume_position, KeyframeResume(0, 0))

        change = store.apply(make_epoch(7))
        self.assertTrue(change.cleared)
        self.assertEqual(store.resume_position, KeyframeResume(7, 0))

        for keyframe_id in range(3):
            change = store.apply(make_added(7, keyframe_id, count=5))
            self.assertEqual(change.added, 1)
            self.assertFalse(change.resync)
        self.assertEqual(store.resume_position, KeyframeResume(7, 3))

        snapshot = store.snapshot()
        self.assertEqual([view.id for view in snapshot.keyframes], [0, 1, 2])
        # The points are the keyframe's own (never merged or transformed), the pose is viser's position and wxyz.
        np.testing.assert_array_equal(snapshot.keyframes[2].points[:, 0], np.arange(2000, 2005, dtype=np.float32))
        np.testing.assert_allclose(snapshot.keyframes[2].position, [2.0, 0.0, 0.0])
        np.testing.assert_allclose(snapshot.keyframes[2].wxyz, [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(snapshot.keyframes[2].colors.shape, (5, 3))

    def test_a_pose_table_moves_keyframes_and_keeps_their_points(self) -> None:
        store = KeyframeStore()
        store.apply(make_epoch(7))
        for keyframe_id in range(3):
            store.apply(make_added(7, keyframe_id))
        before = store.snapshot()

        change = store.apply(make_poses(7, 5, [make_pose(), make_pose(x=1.5, yaw=np.pi / 2), make_pose(x=2.0)]))

        self.assertEqual((change.moved, change.added), (1, 0))  # only keyframe 1 differs from what it was
        after = store.snapshot()
        self.assertEqual(after.graph_version, 5)
        np.testing.assert_allclose(after.keyframes[1].position, [1.5, 0.0, 0.0])
        np.testing.assert_allclose(after.keyframes[1].wxyz, [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
        for index in range(3):
            # The very same arrays: nothing was copied or re-sent.
            self.assertIs(after.keyframes[index].points, before.keyframes[index].points)
        self.assertGreater(after.keyframes[1].pose_version, before.keyframes[1].pose_version)
        self.assertEqual(after.keyframes[0].pose_version, before.keyframes[0].pose_version)

    def test_a_pose_for_a_keyframe_not_received_yet_is_ignored(self) -> None:
        store = KeyframeStore()
        store.apply(make_epoch(7))
        store.apply(make_added(7, 0))
        change = store.apply(make_poses(7, 1, [make_pose(x=9.0), make_pose(x=3.0)]))
        self.assertEqual(change.moved, 1)
        self.assertEqual(store.keyframe_count, 1)
        # Its own message then carries its pose.
        store.apply(make_added(7, 1, pose=make_pose(x=3.5)))
        np.testing.assert_allclose(store.snapshot().keyframes[1].position, [3.5, 0.0, 0.0])

    def test_pose_tables_of_another_epoch_are_ignored(self) -> None:
        store = KeyframeStore()
        store.apply(make_epoch(7))
        store.apply(make_added(7, 0))
        change = store.apply(make_poses(6, 1, [make_pose(x=9.0)]))
        self.assertFalse(change.changed)
        np.testing.assert_allclose(store.snapshot().keyframes[0].position, [0.0, 0.0, 0.0])

    def test_epoch_change_clears_everything_and_restarts_the_ids(self) -> None:
        store = KeyframeStore()
        store.apply(make_epoch(7))
        store.apply(make_added(7, 0))
        store.apply(make_added(7, 1))
        generation = store.snapshot().generation

        change = store.apply(make_epoch(8))

        self.assertTrue(change.cleared)
        self.assertEqual(store.keyframe_count, 0)
        self.assertGreater(store.snapshot().generation, generation)
        self.assertEqual(store.resume_position, KeyframeResume(8, 0))
        store.apply(make_added(8, 0))
        self.assertEqual(store.keyframe_count, 1)

    def test_a_resumed_stream_of_the_same_epoch_changes_nothing(self) -> None:
        store = KeyframeStore()
        store.apply(make_epoch(7))
        store.apply(make_added(7, 0))
        change = store.apply(make_epoch(7))
        self.assertFalse(change.changed)
        self.assertEqual(store.keyframe_count, 1)

    def test_protocol_violations_clear_and_ask_for_a_restart(self) -> None:
        for name, message in (
            ("duplicate", make_added(7, 0)),
            ("gap", make_added(7, 5)),
            ("wrong epoch", make_added(9, 1)),
        ):
            with self.subTest(name):
                store = KeyframeStore()
                store.apply(make_epoch(7))
                store.apply(make_added(7, 0))
                with self.assertLogs("keyframe_store", level="ERROR"):
                    change = store.apply(message)
                self.assertTrue(change.resync)
                self.assertEqual(store.keyframe_count, 0)
                self.assertEqual(store.resume_position, KeyframeResume(0, 0))

    def test_a_malformed_message_leaves_the_store_untouched(self) -> None:
        store = KeyframeStore()
        store.apply(make_epoch(7))
        store.apply(make_added(7, 0))
        bad = make_added(7, 1, pose=make_pose())
        bad.added.pose.qw = 0.0  # a zero quaternion
        with self.assertRaises(ValueError):
            store.apply(bad)
        self.assertEqual(store.keyframe_count, 1)


if __name__ == "__main__":
    unittest.main()
