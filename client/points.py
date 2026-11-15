import sys
from pathlib import Path

import numpy as np

from colormap import rainbow_colormap

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import lidar_pb2  # noqa: E402


def to_viser_points(point_cloud: lidar_pb2.PointCloud3) -> np.ndarray:
    if len(point_cloud.x) == 0:
        return np.empty((0, 3), dtype=np.float32)

    if len(point_cloud.x) != len(point_cloud.y) or len(point_cloud.x) != len(point_cloud.z):
        raise ValueError("PointCloud3 contains mismatched x/y/z array lengths")

    x = np.asarray(point_cloud.x, dtype=np.float32)
    y = np.asarray(point_cloud.y, dtype=np.float32)
    z = np.asarray(point_cloud.z, dtype=np.float32)
    return np.column_stack((x, y, z))


# Mid360 reflectivity is 0 to 255 and `toGRPC` (src/slam/Conversion.cc) copies it into the uint32 `intensity` field
# (map points average it over a voxel, so it stays inside that range). A fixed range keeps a point's colour
# independent of the chunk it arrives in.
INTENSITY_RANGE = (0.0, 255.0)
# Fallback when a cloud has no intensity: colour by height over a fixed range in metres (indoor scale). Heights
# outside the range saturate.
HEIGHT_RANGE_M = (-2.0, 4.0)


def to_viser_colors(points: np.ndarray) -> np.ndarray:
    """Colour points by height over the fixed `HEIGHT_RANGE_M`; each point depends only on its own z."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.uint8)

    z_min, z_max = HEIGHT_RANGE_M
    normalized = np.clip((points[:, 2].astype(np.float32) - z_min) / (z_max - z_min), 0.0, 1.0)

    colors = np.empty((len(points), 3), dtype=np.uint8)
    colors[:, 0] = np.clip(255.0 * normalized, 0, 255).astype(np.uint8)
    colors[:, 1] = np.clip(255.0 * (1.0 - np.abs(normalized - 0.5) * 2.0), 0, 255).astype(np.uint8)
    colors[:, 2] = np.clip(255.0 * (1.0 - normalized), 0, 255).astype(np.uint8)
    return colors


def intensity_to_colors(point_cloud: lidar_pb2.PointCloud3, num_points: int) -> np.ndarray | None:
    """Return rainbow colors derived from intensity over the fixed `INTENSITY_RANGE`, or None if unavailable."""
    if len(point_cloud.intensity) != num_points or num_points == 0:
        return None

    i_min, i_max = INTENSITY_RANGE
    intensity = np.asarray(point_cloud.intensity, dtype=np.float32)
    normalized = 1.0 - np.clip((intensity - i_min) / (i_max - i_min), 0.0, 1.0)
    return rainbow_colormap(normalized)


def to_point_colors(point_cloud: lidar_pb2.PointCloud3, points: np.ndarray) -> np.ndarray:
    """Colours for `points` (from `to_viser_points`): intensity if the cloud has it, else height."""
    colors = intensity_to_colors(point_cloud, len(points))
    return colors if colors is not None else to_viser_colors(points)
