"""Client-side model of the SLAM map built from the `StreamMap` stream.

The model knows nothing about viser or gRPC (a chunk is any object with `epoch`, `first_offset`, `next_offset` and
`points`, such as `slam_pb2.MapStreamChunk`), so it can be tested headless. It is thread safe: a stream thread
applies chunks while a renderer thread takes snapshots.

Storage is in blocks of at most `capacity` points. Points are appended to the single *open* block; when it is full
it is *sealed* and never touched again, then a new open block starts. An append copies only the appended points.
"""

import logging
import threading
from dataclasses import dataclass
from typing import Any, NamedTuple

import numpy as np

from points import to_point_colors, to_viser_points

logger = logging.getLogger(__name__)

DEFAULT_BLOCK_CAPACITY = 200_000


class ResumePosition(NamedTuple):
    """Where the next `MapStreamRequest` should start. `(0, 0)` means from the beginning."""

    epoch: int
    next_offset: int


@dataclass(frozen=True)
class SealedBlock:
    """A full block. Its arrays are read-only and the block is never modified."""

    index: int
    points: np.ndarray
    colors: np.ndarray


@dataclass(frozen=True)
class MapChange:
    """What one `MapStore.apply` did, so a consumer can update incrementally."""

    cleared: bool = False
    """The map was emptied (epoch change or recovery) before anything was appended."""
    appended_points: int = 0
    sealed_blocks: int = 0
    """Blocks sealed by this chunk."""
    resync: bool = False
    """Protocol violation: the stream must be restarted; the store asks for offset 0 with epoch 0."""

    @property
    def changed(self) -> bool:
        return self.cleared or self.appended_points > 0 or self.sealed_blocks > 0


@dataclass(frozen=True)
class MapSnapshot:
    """A consistent view for rendering: the sealed blocks and a copy of the open block's points."""

    generation: int
    """Increases every time the map is cleared; sealed block indices restart at 0 with each generation."""
    sealed: tuple[SealedBlock, ...]
    open_points: np.ndarray
    open_colors: np.ndarray


class MapStore:
    def __init__(self, capacity: int = DEFAULT_BLOCK_CAPACITY) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._capacity = capacity
        self._lock = threading.Lock()
        self._epoch = 0
        self._next_offset = 0
        self._generation = 0
        self._sealed: list[SealedBlock] = []
        self._open_points = np.empty((capacity, 3), dtype=np.float32)
        self._open_colors = np.empty((capacity, 3), dtype=np.uint8)
        self._open_count = 0
        self._violation_logged = False
        self.copied_points = 0
        """Points copied into blocks so far (every appended point once); lets tests check the cost of an append."""

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def resume_position(self) -> ResumePosition:
        with self._lock:
            return ResumePosition(self._epoch, self._next_offset)

    @property
    def point_count(self) -> int:
        with self._lock:
            return len(self._sealed) * self._capacity + self._open_count

    def apply(self, chunk: Any) -> MapChange:
        """Apply one `MapStreamChunk`; see the module docstring and `grpc/slam.proto` for the contract."""
        # Convert first: a malformed chunk raises before the store is touched.
        points = to_viser_points(chunk.points)
        colors = to_point_colors(chunk.points, points)
        return self.apply_arrays(chunk.epoch, chunk.first_offset, chunk.next_offset, points, colors)

    def apply_arrays(
        self, epoch: int, first_offset: int, next_offset: int, points: np.ndarray, colors: np.ndarray
    ) -> MapChange:
        with self._lock:
            cleared = False
            if epoch != self._epoch:
                self._clear_locked()
                self._epoch = epoch
                self._next_offset = 0
                cleared = True

            if first_offset < self._next_offset:
                # Same epoch and a chunk that starts before what we hold: applying it would render duplicates.
                if not self._violation_logged:
                    self._violation_logged = True
                    logger.error(
                        "Map stream protocol violation: epoch %d chunk starts at offset %d but %d was already applied;"
                        " clearing the map and restarting from offset 0",
                        epoch,
                        first_offset,
                        self._next_offset,
                    )
                self._clear_locked()
                self._epoch = 0
                self._next_offset = 0
                return MapChange(cleared=True, resync=True)

            sealed_before = len(self._sealed)
            self._append_locked(points, colors)
            self._next_offset = next_offset
            return MapChange(
                cleared=cleared, appended_points=len(points), sealed_blocks=len(self._sealed) - sealed_before
            )

    def snapshot(self) -> MapSnapshot:
        """A consistent view: at most one block of points is copied, however large the map is."""
        with self._lock:
            return MapSnapshot(
                self._generation,
                tuple(self._sealed),
                self._open_points[: self._open_count].copy(),
                self._open_colors[: self._open_count].copy(),
            )

    def all_points(self) -> np.ndarray:
        """The whole map as one (N, 3) float32 array. O(map): for tests and tools, not for per-chunk use."""
        with self._lock:
            parts = [block.points for block in self._sealed] + [self._open_points[: self._open_count]]
            return np.concatenate(parts, axis=0)

    def all_colors(self) -> np.ndarray:
        """The whole map's colours as one (N, 3) uint8 array. O(map), like `all_points`."""
        with self._lock:
            parts = [block.colors for block in self._sealed] + [self._open_colors[: self._open_count]]
            return np.concatenate(parts, axis=0)

    def _clear_locked(self) -> None:
        self._generation += 1
        self._sealed = []
        # The open block's storage is reused: snapshots copy it, and sealed arrays are separate objects.
        self._open_count = 0

    def _append_locked(self, points: np.ndarray, colors: np.ndarray) -> None:
        total = len(points)
        done = 0
        while done < total:
            take = min(self._capacity - self._open_count, total - done)
            end = self._open_count + take
            self._open_points[self._open_count : end] = points[done : done + take]
            self._open_colors[self._open_count : end] = colors[done : done + take]
            self._open_count = end
            self.copied_points += take
            done += take
            if self._open_count == self._capacity:
                self._seal_locked()

    def _seal_locked(self) -> None:
        self._open_points.flags.writeable = False
        self._open_colors.flags.writeable = False
        self._sealed.append(SealedBlock(len(self._sealed), self._open_points, self._open_colors))
        self._open_points = np.empty((self._capacity, 3), dtype=np.float32)
        self._open_colors = np.empty((self._capacity, 3), dtype=np.uint8)
        self._open_count = 0
