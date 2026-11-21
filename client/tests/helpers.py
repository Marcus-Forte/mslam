"""Builders shared by the client tests."""

import sys
from pathlib import Path

CLIENT_DIR = Path(__file__).resolve().parent.parent
if str(CLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(CLIENT_DIR))
if str(CLIENT_DIR / "proto_gen") not in sys.path:
    sys.path.insert(0, str(CLIENT_DIR / "proto_gen"))

from proto_gen import lidar_pb2, slam_pb2  # noqa: E402


def make_cloud(count: int, start: int = 0) -> lidar_pb2.PointCloud3:
    """A cloud whose point k (global index start + k) has x = index, y = 2 * index, z = (index % 60) / 10,
    intensity = index % 256, so any reordering, duplication or rechunking is visible in the data."""
    cloud = lidar_pb2.PointCloud3()
    for index in range(start, start + count):
        cloud.x.append(float(index))
        cloud.y.append(2.0 * index)
        cloud.z.append((index % 60) / 10.0)
        cloud.intensity.append(index % 256)
    return cloud


def make_chunk(epoch: int, first: int, next_: int, count: int = 0, start: int = 0) -> slam_pb2.MapStreamChunk:
    return slam_pb2.MapStreamChunk(epoch=epoch, first_offset=first, next_offset=next_, points=make_cloud(count, start))


def make_pose(x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0) -> slam_pb2.PoseQuat:
    """A `PoseQuat` at (x, y, z) rotated by `yaw` about z (unit quaternion, w first)."""
    import math

    return slam_pb2.PoseQuat(x=x, y=y, z=z, qw=math.cos(yaw / 2), qx=0.0, qy=0.0, qz=math.sin(yaw / 2))


def make_epoch(epoch: int) -> slam_pb2.KeyframeStreamMessage:
    return slam_pb2.KeyframeStreamMessage(epoch_reset=slam_pb2.KeyframeEpoch(epoch=epoch))


def make_added(epoch: int, keyframe_id: int, count: int = 4, pose: slam_pb2.PoseQuat | None = None):
    """Keyframe `keyframe_id` with `count` points (see `make_cloud`; x counts up from 1000 * id)."""
    return slam_pb2.KeyframeStreamMessage(
        added=slam_pb2.KeyframeAdded(
            epoch=epoch,
            id=keyframe_id,
            pose=pose if pose is not None else make_pose(x=float(keyframe_id)),
            cloud=make_cloud(count, start=1000 * keyframe_id),
        )
    )


def make_poses(epoch: int, graph_version: int, poses: list[slam_pb2.PoseQuat]):
    return slam_pb2.KeyframeStreamMessage(
        poses=slam_pb2.KeyframePoses(
            epoch=epoch,
            graph_version=graph_version,
            poses=[slam_pb2.KeyframePoseEntry(id=index, pose=pose) for index, pose in enumerate(poses)],
        )
    )
