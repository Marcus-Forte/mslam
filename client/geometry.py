import numpy as np


def euler_xyz_to_wxyz(rx: float, ry: float, rz: float) -> np.ndarray:
    half_rx = rx * 0.5
    half_ry = ry * 0.5
    half_rz = rz * 0.5

    cx = np.cos(half_rx)
    sx = np.sin(half_rx)
    cy = np.cos(half_ry)
    sy = np.sin(half_ry)
    cz = np.cos(half_rz)
    sz = np.sin(half_rz)

    qx = np.array([cx, sx, 0.0, 0.0], dtype=np.float32)
    qy = np.array([cy, 0.0, sy, 0.0], dtype=np.float32)
    qz = np.array([cz, 0.0, 0.0, sz], dtype=np.float32)

    # Match the C++ transform construction order: roll * pitch * yaw.
    return quaternion_multiply(quaternion_multiply(qx, qy), qz)


def quaternion_multiply(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float32,
    )
