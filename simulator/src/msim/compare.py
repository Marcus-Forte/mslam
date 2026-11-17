"""Compare an estimated trajectory with the ground-truth log.

The estimate's frame is mslam's: origin at the first sensor pose, yaw 0, z up. For a
level start that is `T0^-1 * T_gt`; the general case used here keeps the world z axis
and removes only the initial yaw and position:  p_est = Rz(-yaw0)(p - p0),  R_est = Rz(-yaw0) R.
mslam's Euler angles are R = Rx(roll) Ry(pitch) Rz(yaw) (Transform.hh), not the aerospace ZYX.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import rot_from_quat, rot_z, rpy_from_rot, so3_log


@dataclass
class Trajectory:
    t_ns: np.ndarray
    p: np.ndarray     # (N,3)
    R: np.ndarray     # (N,3,3)


def load_gt(path: str) -> Trajectory:
    d = np.genfromtxt(path, delimiter=",", names=True)
    q = np.stack([d["qw"], d["qx"], d["qy"], d["qz"]], -1)
    return Trajectory(d["timestamp_ns"].astype(np.int64), np.stack([d["x"], d["y"], d["z"]], -1), rot_from_quat(q))


def mslam_euler_to_rot(rx: float, ry: float, rz: float) -> np.ndarray:
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def load_bench_trajectory(path: str) -> Trajectory:
    """`bench_slam --trajectory` CSV (timestamp_ns,x,y,z,roll,pitch,yaw,...)."""
    d = np.genfromtxt(path, delimiter=",", names=True)
    R = np.stack([mslam_euler_to_rot(a, b, c) for a, b, c in zip(d["roll"], d["pitch"], d["yaw"])])
    return Trajectory(d["timestamp_ns"].astype(np.int64), np.stack([d["x"], d["y"], d["z"]], -1), R)


def gt_in_slam_frame(gt: Trajectory) -> Trajectory:
    _, _, yaw0 = rpy_from_rot(gt.R[0])
    A = rot_z(-yaw0)
    return Trajectory(gt.t_ns, (gt.p - gt.p[0]) @ A.T, np.einsum("ij,njk->nik", A, gt.R))


@dataclass
class Errors:
    n: int
    pos_err: np.ndarray   # m
    rot_err: np.ndarray   # rad
    path_length: float

    def summary(self) -> str:
        return (f"{self.n} poses matched; position error m: mean {self.pos_err.mean():.4f} rms "
                f"{np.sqrt((self.pos_err**2).mean()):.4f} max {self.pos_err.max():.4f} final {self.pos_err[-1]:.4f}; "
                f"rotation error mrad: mean {1e3 * self.rot_err.mean():.2f} max {1e3 * self.rot_err.max():.2f} "
                f"final {1e3 * self.rot_err[-1]:.2f}; GT path length {self.path_length:.2f} m "
                f"(final drift {100 * self.pos_err[-1] / max(self.path_length, 1e-9):.2f} % of path)")


def compare(gt: Trajectory, est: Trajectory, est_offset_ns: int = 47_995_000) -> Errors:
    """Match each estimate to the GT row at `est.t + est_offset_ns` (the estimate is stamped with the
    first point, the GT with the last: 47.995 ms for 9600 points at 200 kHz)."""
    ref = gt_in_slam_frame(gt)
    idx = {int(t): i for i, t in enumerate(ref.t_ns)}
    pairs = [(i, idx[int(t) + est_offset_ns]) for i, t in enumerate(est.t_ns) if int(t) + est_offset_ns in idx]
    if not pairs:
        raise ValueError("no timestamps in common between the estimate and the ground truth")
    ei, gi = (np.array(a) for a in zip(*pairs))
    perr = np.linalg.norm(est.p[ei] - ref.p[gi], axis=1)
    rerr = np.array([np.linalg.norm(so3_log(ref.R[g].T @ est.R[e])) for e, g in zip(ei, gi)])
    path = float(np.linalg.norm(np.diff(ref.p, axis=0), axis=1).sum())
    return Errors(len(pairs), perr, rerr, path)
