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


def to_viser_colors(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.uint8)

    z_values = points[:, 2]
    z_min = float(z_values.min())
    z_max = float(z_values.max())
    z_span = max(z_max - z_min, 1e-6)
    normalized = (z_values - z_min) / z_span

    colors = np.empty((len(points), 3), dtype=np.uint8)
    colors[:, 0] = np.clip(255.0 * normalized, 0, 255).astype(np.uint8)
    colors[:, 1] = np.clip(255.0 * (1.0 - np.abs(normalized - 0.5) * 2.0), 0, 255).astype(np.uint8)
    colors[:, 2] = np.clip(255.0 * (1.0 - normalized), 0, 255).astype(np.uint8)
    return colors


def intensity_to_colors(point_cloud: lidar_pb2.PointCloud3, num_points: int) -> np.ndarray | None:
    """Return rainbow colors derived from intensity, or None if unavailable."""
    if len(point_cloud.intensity) != num_points or num_points == 0:
        return None

    intensity = np.asarray(point_cloud.intensity, dtype=np.float32)
    i_min = float(intensity.min())
    i_max = float(intensity.max())
    normalized = 1.0 - (intensity - i_min) / max(i_max - i_min, 1e-6)
    return rainbow_colormap(normalized)
