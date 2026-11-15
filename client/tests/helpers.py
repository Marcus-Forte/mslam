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
