"""Client-side model of the pose-graph map built from the `StreamKeyframes` stream.

Like `map_store.py` it knows nothing about viser or gRPC (a message is any object with `WhichOneof("payload")` and
the fields of `slam_pb2.KeyframeStreamMessage`), so it can be tested headless. It is thread safe: a stream thread
applies messages while a renderer thread takes snapshots.

A keyframe's points arrive once, in the keyframe's own frame, and never change. Only its pose changes, when the
server's optimization moves the keyframe: the store keeps the arrays and replaces just the pose, so a consumer sees
a pose change as one small update and the points never travel (or get copied) again.
"""

import logging
import threading
from dataclasses import dataclass
from typing import Any, NamedTuple

import numpy as np

from points import to_point_colors, to_viser_points

logger = logging.getLogger(__name__)


class KeyframeResume(NamedTuple):
    """Where the next `KeyframeStreamRequest` should start. `(0, 0)` means from the beginning."""

    epoch: int
    next_keyframe_id: int


@dataclass(frozen=True)
class KeyframeView:
    """One keyframe. The arrays are shared between views of the same keyframe and must not be modified."""

    id: int
    points: np.ndarray
    """(N, 3) float32, in the keyframe's own frame."""
    colors: np.ndarray
    position: np.ndarray
    """(3,) float64: the keyframe's origin in the map frame (viser `position`)."""
    wxyz: np.ndarray
    """(4,) float64 unit quaternion, w first (viser `wxyz`)."""
    pose_version: int
    """Store-wide counter, increases each time this keyframe's pose is set; lets a renderer see what moved."""


@dataclass(frozen=True)
class KeyframeChange:
    """What one `KeyframeStore.apply` did, so a consumer can update incrementally."""

    cleared: bool = False
    """Every keyframe was dropped (epoch change or recovery) before anything was added."""
    added: int = 0
    moved: int = 0
    """Keyframes whose pose changed."""
    resync: bool = False
    """Protocol violation: the stream must be restarted; the store asks for epoch 0, keyframe 0."""

    @property
    def changed(self) -> bool:
        return self.cleared or self.added > 0 or self.moved > 0


@dataclass(frozen=True)
class KeyframeSnapshot:
    generation: int
    """Increases every time the store is cleared; keyframe ids restart at 0 with each generation."""
    keyframes: tuple[KeyframeView, ...]
    """Index = keyframe id."""
    graph_version: int


def _pose_arrays(pose: Any) -> tuple[np.ndarray, np.ndarray]:
    position = np.array([pose.x, pose.y, pose.z], dtype=np.float64)
    wxyz = np.array([pose.qw, pose.qx, pose.qy, pose.qz], dtype=np.float64)
    norm = np.linalg.norm(wxyz)
    if not np.isfinite(norm) or norm == 0.0:
        raise ValueError("pose quaternion is not finite or is zero")
    return position, wxyz / norm


class KeyframeStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._epoch = 0
        self._generation = 0
        self._keyframes: list[KeyframeView] = []
        self._pose_counter = 0
        self._graph_version = 0
        self._violation_logged = False

    @property
    def resume_position(self) -> KeyframeResume:
        with self._lock:
            return KeyframeResume(self._epoch, len(self._keyframes))

    @property
    def keyframe_count(self) -> int:
        with self._lock:
            return len(self._keyframes)

    def apply(self, message: Any) -> KeyframeChange:
        """Apply one `KeyframeStreamMessage`; see `grpc/slam.proto` for the contract."""
        kind = message.WhichOneof("payload")
        if kind == "epoch_reset":
            return self._apply_epoch(message.epoch_reset.epoch)
        if kind == "added":
            added = message.added
            # Convert first: a malformed message raises before the store is touched.
            points = to_viser_points(added.cloud)
            colors = to_point_colors(added.cloud, points)
            position, wxyz = _pose_arrays(added.pose)
            return self._apply_added(added.epoch, added.id, points, colors, position, wxyz)
        if kind == "poses":
            poses = message.poses
            entries = [(entry.id, *_pose_arrays(entry.pose)) for entry in poses.poses]
            return self._apply_poses(poses.epoch, poses.graph_version, entries)
        return KeyframeChange()  # a payload this client does not know: ignore it

    def snapshot(self) -> KeyframeSnapshot:
        """A consistent view; the arrays are shared, so this is O(keyframes) pointers."""
        with self._lock:
            return KeyframeSnapshot(self._generation, tuple(self._keyframes), self._graph_version)

    def _apply_epoch(self, epoch: int) -> KeyframeChange:
        with self._lock:
            if epoch == self._epoch:
                return KeyframeChange()  # a resumed stream of the epoch we hold
            self._clear_locked()
            self._epoch = epoch
            return KeyframeChange(cleared=True)

    def _apply_added(
        self,
        epoch: int,
        keyframe_id: int,
        points: np.ndarray,
        colors: np.ndarray,
        position: np.ndarray,
        wxyz: np.ndarray,
    ) -> KeyframeChange:
        with self._lock:
            if epoch != self._epoch or keyframe_id != len(self._keyframes):
                return self._violation_locked(
                    f"keyframe {keyframe_id} of epoch {epoch} while holding {len(self._keyframes)} of epoch {self._epoch}"
                )
            points.flags.writeable = False
            colors.flags.writeable = False
            self._pose_counter += 1
            self._keyframes.append(KeyframeView(keyframe_id, points, colors, position, wxyz, self._pose_counter))
            return KeyframeChange(added=1)

    def _apply_poses(
        self, epoch: int, graph_version: int, entries: list[tuple[int, np.ndarray, np.ndarray]]
    ) -> KeyframeChange:
        with self._lock:
            if epoch != self._epoch:
                logger.debug("Ignoring a pose table of epoch %d (holding epoch %d)", epoch, self._epoch)
                return KeyframeChange()
            moved = 0
            for keyframe_id, position, wxyz in entries:
                if keyframe_id >= len(self._keyframes):
                    continue  # a keyframe this client has not received (yet): its own message carries its pose
                old = self._keyframes[keyframe_id]
                if np.array_equal(old.position, position) and np.array_equal(old.wxyz, wxyz):
                    continue
                self._pose_counter += 1
                self._keyframes[keyframe_id] = KeyframeView(
                    old.id, old.points, old.colors, position, wxyz, self._pose_counter
                )
                moved += 1
            self._graph_version = max(self._graph_version, graph_version)
            return KeyframeChange(moved=moved)

    def _violation_locked(self, what: str) -> KeyframeChange:
        if not self._violation_logged:
            self._violation_logged = True
            logger.error("Keyframe stream protocol violation (%s); clearing and restarting from keyframe 0", what)
        self._clear_locked()
        self._epoch = 0
        return KeyframeChange(cleared=True, resync=True)

    def _clear_locked(self) -> None:
        self._generation += 1
        self._keyframes = []
        self._graph_version = 0
