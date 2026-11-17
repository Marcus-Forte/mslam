"""Rotation helpers (vectorised over leading axes).

World frame: z up, right-handed, gravity is -z. Sensor/body frame (Livox
convention): x forward, y left, z up. Quaternions are (w, x, y, z).

Euler convention (used by scripted trajectories and `kin_from_euler`):
`R = Rz(yaw) @ Ry(-pitch_up) @ Rx(roll)`: yaw about world z, then pitch about
the yawed y axis (positive = x axis pointing UP, i.e. a negative right-hand
rotation about y), then roll about the body x axis (positive = right-hand
rotation about x = left side up, "roll right"). Roll 0 leaves the old
two-angle parametrisation unchanged bit for bit. The interactive controller
does not use Euler angles at all: it integrates `R` directly (full SO(3)).
"""

from __future__ import annotations

import numpy as np

GRAVITY = 9.80665  # m/s^2, the constant mslam uses (ImuMath / imu_acceleration_scale)
G_WORLD = np.array([0.0, 0.0, -GRAVITY])


def rot_z(a):
    a = np.asarray(a, dtype=np.float64)
    c, s = np.cos(a), np.sin(a)
    z, o = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, -s, z], -1), np.stack([s, c, z], -1), np.stack([z, z, o], -1)], -2)


def rot_y(a):
    a = np.asarray(a, dtype=np.float64)
    c, s = np.cos(a), np.sin(a)
    z, o = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, z, s], -1), np.stack([z, o, z], -1), np.stack([-s, z, c], -1)], -2)


def rot_x(a):
    a = np.asarray(a, dtype=np.float64)
    c, s = np.cos(a), np.sin(a)
    z, o = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([o, z, z], -1), np.stack([z, c, -s], -1), np.stack([z, s, c], -1)], -2)


def rot_from_yaw_pitch(yaw, pitch_up, roll=0.0):
    """Body-to-world rotation `Rz(yaw) Ry(-pitch_up) Rx(roll)` (rad). With roll == 0 the Rx factor is skipped."""
    R = rot_z(yaw) @ rot_y(-np.asarray(pitch_up, dtype=np.float64))
    if np.any(np.asarray(roll) != 0.0):
        R = R @ rot_x(roll)
    return R


def exp_so3(w):
    """Rotation matrix of the rotation vector `w` (Rodrigues)."""
    w = np.asarray(w, dtype=np.float64)
    ang = float(np.sqrt(w @ w))
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if ang < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + (np.sin(ang) / ang) * K + ((1.0 - np.cos(ang)) / (ang * ang)) * K @ K


def orthonormalize(R):
    """Gram-Schmidt on the columns (x kept, z = x cross y, then y = z cross x)."""
    x = R[:, 0] / np.linalg.norm(R[:, 0])
    z = np.cross(x, R[:, 1])
    z /= np.linalg.norm(z)
    return np.stack([x, np.cross(z, x), z], axis=1)


def angular_velocity_body(yaw, yaw_rate, pitch_up, pitch_rate, roll=0.0, roll_rate=0.0):
    """Body-frame angular velocity of `rot_from_yaw_pitch` (exact, not differenced).

    With beta = -pitch_up:  R = Rz(yaw) Ry(beta) Rx(roll),
    omega_world = yaw_rate z + beta_rate (Rz y) + roll_rate (Rz Ry x);  omega_body = R^T omega_world.
    """
    yaw = np.asarray(yaw, dtype=np.float64)
    R = rot_from_yaw_pitch(yaw, pitch_up, roll)
    z = np.zeros_like(yaw)
    o = np.ones_like(yaw)
    y_axis = np.stack([-np.sin(yaw), np.cos(yaw), z], -1)  # Rz(yaw) @ [0, 1, 0]
    omega_w = np.asarray(yaw_rate)[..., None] * np.stack([z, z, o], -1) - np.asarray(pitch_rate)[..., None] * y_axis
    if np.any(np.asarray(roll_rate) != 0.0):
        x_axis = (rot_z(yaw) @ rot_y(-np.asarray(pitch_up, dtype=np.float64)))[..., :, 0]  # Rz Ry x
        omega_w = omega_w + np.asarray(roll_rate)[..., None] * x_axis
    return np.einsum("...ji,...j->...i", R, omega_w)


def quat_from_rot(R):
    """Rotation matrices (...,3,3) to unit quaternions (...,4), w >= 0."""
    R = np.asarray(R, dtype=np.float64)
    m00, m11, m22 = R[..., 0, 0], R[..., 1, 1], R[..., 2, 2]
    tr = m00 + m11 + m22
    q = np.empty(R.shape[:-2] + (4,))
    # Shepperd's method: pick the largest of (tr, m00, m11, m22) for stability.
    cand = np.stack([tr, m00, m11, m22], -1)
    k = np.argmax(cand, -1)
    flat_R = R.reshape(-1, 3, 3)
    flat_q = q.reshape(-1, 4)
    flat_k = k.reshape(-1)
    for i in range(flat_R.shape[0]):
        r = flat_R[i]
        kk = flat_k[i]
        if kk == 0:
            s = 2.0 * np.sqrt(1.0 + r[0, 0] + r[1, 1] + r[2, 2])
            w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
        elif kk == 1:
            s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
        elif kk == 2:
            s = 2.0 * np.sqrt(1.0 - r[0, 0] + r[1, 1] - r[2, 2])
            w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 - r[0, 0] - r[1, 1] + r[2, 2])
            w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
        if w < 0:
            w, x, y, z = -w, -x, -y, -z
        flat_q[i] = (w, x, y, z)
    return q


def rot_from_quat(q):
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - w * z)
    R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y)
    R[..., 2, 1] = 2 * (y * z + w * x)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def nlerp(q0, q1, f):
    """Normalised lerp of quaternions along the short arc (error << 1e-6 rad for 1 ms steps)."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    sign = np.where(np.sum(q0 * q1, -1, keepdims=True) < 0.0, -1.0, 1.0)
    q = (1.0 - np.asarray(f)[..., None]) * q0 + np.asarray(f)[..., None] * sign * q1
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def rpy_from_rot(R):
    """Roll, pitch, yaw (rad) with R = Rz(yaw) Ry(pitch) Rx(roll) (the usual aerospace ZYX)."""
    R = np.asarray(R, dtype=np.float64)
    pitch = np.arcsin(np.clip(-R[..., 2, 0], -1.0, 1.0))
    roll = np.arctan2(R[..., 2, 1], R[..., 2, 2])
    yaw = np.arctan2(R[..., 1, 0], R[..., 0, 0])
    return roll, pitch, yaw


def so3_log(R):
    """Rotation vector of a single rotation matrix."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-9:
        return 0.5 * v
    return v * ang / (2.0 * np.sin(ang))
