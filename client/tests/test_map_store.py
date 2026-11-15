import logging
import unittest

import numpy as np

from tests.helpers import make_chunk
from map_store import MapStore, ResumePosition


class ApplyTest(unittest.TestCase):
    def test_in_order_chunks_are_appended_and_advance_the_resume_position(self) -> None:
        store = MapStore(capacity=100)
        self.assertEqual(store.resume_position, ResumePosition(0, 0))

        change = store.apply(make_chunk(7, 0, 0))  # the empty announcement
        self.assertTrue(change.cleared)
        self.assertEqual(store.resume_position, ResumePosition(7, 0))

        change = store.apply(make_chunk(7, 0, 3, count=10, start=0))
        self.assertFalse(change.cleared)
        self.assertEqual(change.appended_points, 10)
        self.assertEqual(store.resume_position, ResumePosition(7, 3))

        store.apply(make_chunk(7, 3, 5, count=5, start=10))
        self.assertEqual(store.resume_position, ResumePosition(7, 5))
        self.assertEqual(store.point_count, 15)
        np.testing.assert_array_equal(store.all_points()[:, 0], np.arange(15, dtype=np.float32))

    def test_trailing_empty_increments_skipped_by_the_server(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 0))
        store.apply(make_chunk(7, 0, 2, count=4))
        # Increments 2..4 were empty and are not sent alone: the next data chunk starts at 5, which is not a gap.
        change = store.apply(make_chunk(7, 5, 6, count=3, start=4))
        self.assertFalse(change.resync)
        self.assertEqual(store.resume_position, ResumePosition(7, 6))
        self.assertEqual(store.point_count, 7)

    def test_first_offset_equal_to_next_offset_is_accepted(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 4, count=2))
        change = store.apply(make_chunk(7, 4, 4))  # announcement of a resumed stream
        self.assertFalse(change.changed)
        self.assertFalse(change.resync)
        self.assertEqual(store.resume_position, ResumePosition(7, 4))

    def test_epoch_change_clears_and_restarts(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 3, count=10))
        change = store.apply(make_chunk(8, 0, 0))
        self.assertTrue(change.cleared)
        self.assertEqual(store.point_count, 0)
        self.assertEqual(store.resume_position, ResumePosition(8, 0))
        store.apply(make_chunk(8, 0, 1, count=2, start=100))
        np.testing.assert_array_equal(store.all_points()[:, 0], [100.0, 101.0])
        self.assertEqual(store.resume_position, ResumePosition(8, 1))

    def test_in_flight_old_epoch_chunk_then_new_epoch_announcement(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 0))
        store.apply(make_chunk(7, 0, 2, count=5))
        store.apply(make_chunk(7, 2, 3, count=5, start=5))  # in flight when the reset happened
        store.apply(make_chunk(9, 0, 0))  # the announcement
        self.assertEqual(store.point_count, 0)
        self.assertEqual(len(store.all_points()), 0)
        self.assertEqual(store.resume_position, ResumePosition(9, 0))

    def test_resumed_stream_with_the_same_epoch_keeps_the_map(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 3, count=10))
        change = store.apply(make_chunk(7, 3, 3))
        self.assertFalse(change.cleared)
        self.assertEqual(store.point_count, 10)

    def test_server_restart_with_a_new_epoch_clears_the_stored_map(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 3, count=10))
        change = store.apply(make_chunk(12345, 0, 0))
        self.assertTrue(change.cleared)
        self.assertEqual(store.point_count, 0)

    def test_protocol_violation_clears_and_asks_for_a_restart_from_zero(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 5, count=10))
        with self.assertLogs("map_store", level=logging.ERROR) as logs:
            change = store.apply(make_chunk(7, 3, 6, count=4))  # starts before the 5 we hold
        self.assertEqual(len(logs.records), 1)
        self.assertTrue(change.resync)
        self.assertTrue(change.cleared)
        self.assertEqual(store.point_count, 0)  # no duplicates were rendered
        self.assertEqual(store.resume_position, ResumePosition(0, 0))

        # The restarted stream (any epoch) is taken from scratch.
        store.apply(make_chunk(7, 0, 0))
        store.apply(make_chunk(7, 0, 5, count=10))
        self.assertEqual(store.point_count, 10)
        self.assertEqual(store.resume_position, ResumePosition(7, 5))

    def test_protocol_violation_is_logged_once(self) -> None:
        store = MapStore(capacity=100)
        with self.assertLogs("map_store", level=logging.ERROR) as logs:
            for _ in range(3):
                store.apply(make_chunk(7, 0, 5, count=2))
                store.apply(make_chunk(7, 1, 2))
        self.assertEqual(len(logs.records), 1)

    def test_malformed_chunk_leaves_the_store_untouched(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 1, count=3))
        bad = make_chunk(7, 1, 2, count=3)
        del bad.points.y[:]
        with self.assertRaises(ValueError):
            store.apply(bad)
        self.assertEqual(store.resume_position, ResumePosition(7, 1))
        self.assertEqual(store.point_count, 3)


class BlockTest(unittest.TestCase):
    def test_seals_at_capacity_and_never_touches_sealed_blocks_again(self) -> None:
        store = MapStore(capacity=10)
        store.apply(make_chunk(7, 0, 0))
        change = store.apply(make_chunk(7, 0, 1, count=25))  # one chunk spanning two seals
        self.assertEqual(change.sealed_blocks, 2)
        snapshot = store.snapshot()
        self.assertEqual([block.index for block in snapshot.sealed], [0, 1])
        self.assertEqual(len(snapshot.open_points), 5)

        frozen = [(block.points.copy(), block.colors.copy()) for block in snapshot.sealed]
        for k in range(10):
            store.apply(make_chunk(7, 1 + k, 2 + k, count=3, start=25 + 3 * k))
        after = store.snapshot()
        self.assertEqual(len(after.sealed), 5)  # 55 points
        for block, (points, colors) in zip(after.sealed, frozen):
            np.testing.assert_array_equal(block.points, points)
            np.testing.assert_array_equal(block.colors, colors)
            self.assertFalse(block.points.flags.writeable)
        # The very same arrays, not copies.
        self.assertIs(after.sealed[0].points, snapshot.sealed[0].points)
        np.testing.assert_array_equal(store.all_points()[:, 0], np.arange(55, dtype=np.float32))

    def test_exactly_full_block_is_sealed_and_the_open_block_is_empty(self) -> None:
        store = MapStore(capacity=10)
        store.apply(make_chunk(7, 0, 1, count=10))
        snapshot = store.snapshot()
        self.assertEqual(len(snapshot.sealed), 1)
        self.assertEqual(len(snapshot.open_points), 0)

    def test_clear_bumps_the_generation_and_empties_the_snapshot(self) -> None:
        store = MapStore(capacity=10)
        store.apply(make_chunk(7, 0, 1, count=25))
        before = store.snapshot()
        store.apply(make_chunk(8, 0, 0))
        after = store.snapshot()
        self.assertNotEqual(before.generation, after.generation)
        self.assertEqual(len(after.sealed), 0)
        self.assertEqual(len(after.open_points), 0)
        self.assertEqual(len(before.sealed), 2)  # an old snapshot stays valid

    def test_snapshot_open_block_is_a_copy(self) -> None:
        store = MapStore(capacity=10)
        store.apply(make_chunk(7, 0, 1, count=3))
        snapshot = store.snapshot()
        store.apply(make_chunk(7, 1, 2, count=3, start=3))
        self.assertEqual(len(snapshot.open_points), 3)


class ChunkingTest(unittest.TestCase):
    TOTAL = 137

    def _deliver(self, sizes: list[int], capacity: int) -> MapStore:
        """Deliver TOTAL points, one increment per point, grouped into chunks of the given increment counts."""
        store = MapStore(capacity=capacity)
        store.apply(make_chunk(7, 0, 0))
        offset = 0
        for size in sizes:
            size = min(size, self.TOTAL - offset)
            if size <= 0:
                break
            store.apply(make_chunk(7, offset, offset + size, count=size, start=offset))
            offset += size
        self.assertEqual(offset, self.TOTAL)
        return store

    def test_same_increments_in_different_chunkings_give_identical_points_and_colors(self) -> None:
        one_by_one = self._deliver([1] * self.TOTAL, capacity=20)
        big = self._deliver([self.TOTAL], capacity=20)
        mixed = self._deliver([1, 5, 40, 2, 100], capacity=20)
        other_capacity = self._deliver([7] * 20, capacity=64)
        for other in (big, mixed, other_capacity):
            np.testing.assert_array_equal(one_by_one.all_points(), other.all_points())
            np.testing.assert_array_equal(one_by_one.all_colors(), other.all_colors())
            self.assertEqual(one_by_one.resume_position, other.resume_position)

    def test_resume_after_a_disconnect_gives_the_same_map(self) -> None:
        whole = self._deliver([self.TOTAL], capacity=20)

        store = MapStore(capacity=20)
        store.apply(make_chunk(7, 0, 0))
        store.apply(make_chunk(7, 0, 60, count=60))
        self.assertEqual(store.resume_position, ResumePosition(7, 60))
        # Reconnect: the server announces the start (60) and continues from there.
        store.apply(make_chunk(7, 60, 60))
        store.apply(make_chunk(7, 60, self.TOTAL, count=self.TOTAL - 60, start=60))
        np.testing.assert_array_equal(store.all_points(), whole.all_points())
        np.testing.assert_array_equal(store.all_colors(), whole.all_colors())


class CostTest(unittest.TestCase):
    def test_copied_points_per_append_do_not_grow_with_the_map(self) -> None:
        capacity = 1_000
        store = MapStore(capacity=capacity)
        store.apply(make_chunk(7, 0, 0))
        per_append: list[int] = []
        for k in range(1_200):
            before = store.copied_points
            store.apply(make_chunk(7, k, k + 1, count=8, start=8 * k))
            per_append.append(store.copied_points - before)
        self.assertEqual(store.point_count, 1_200 * 8)
        self.assertGreater(len(store.snapshot().sealed), 8)
        # Every append copies exactly the points it was given, however large the map is; a concatenating
        # implementation would copy the whole map each time.
        self.assertEqual(set(per_append), {8})

    def test_a_snapshot_copies_at_most_one_block(self) -> None:
        store = MapStore(capacity=100)
        store.apply(make_chunk(7, 0, 1, count=1_050))
        snapshot = store.snapshot()
        self.assertLessEqual(len(snapshot.open_points), store.capacity)
        self.assertEqual(len(snapshot.open_points), 50)


if __name__ == "__main__":
    unittest.main()
