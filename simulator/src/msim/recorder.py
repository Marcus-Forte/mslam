"""Ground-truth CSV and `.pbscan` recording writers (sinks for `SimEngine`)."""

from __future__ import annotations

import struct
from pathlib import Path

from ._proto import ensure_generated

ensure_generated()
import recording_pb2  # noqa: E402

from .engine import GroundTruth  # noqa: E402
from .geometry import rpy_from_rot  # noqa: E402
from .sensors import ImuSample, ScanData  # noqa: E402
from .streams import imu_to_proto, scan_to_proto  # noqa: E402

GT_HEADER = "timestamp_ns,seq,x,y,z,qw,qx,qy,qz,roll,pitch,yaw"


class GroundTruthWriter:
    """One row per scan: the true sensor pose at the scan's LAST point time.

    Pose of the sensor frame in the scene (world) frame, z up. Quaternion (w,x,y,z) and
    roll/pitch/yaw in rad with R = Rz(yaw) Ry(pitch) Rx(roll). Positions in metres.
    NOTE: mslam's own Euler convention is R = Rx Ry Rz, and its world frame starts at
    the first sensor pose; use `msim compare`, which handles the alignment.
    """

    def __init__(self, path: str | Path) -> None:
        self._f = open(path, "w", buffering=1)
        self._f.write(GT_HEADER + "\n")

    def __call__(self, scan: ScanData, gt: GroundTruth) -> None:
        r, p, y = rpy_from_rot(gt.R)
        self._f.write("%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n"
                      % (gt.t_ns, gt.seq, *gt.p, *gt.q, r, p, y))

    def close(self) -> None:
        self._f.close()


def iter_pbscan(path: str | Path):
    """`RecordingEntry` messages of a msensor `.pbscan` ([uint64 LE size][bytes] repeated)."""
    with open(path, "rb") as f:
        while True:
            head = f.read(8)
            if len(head) < 8:
                return
            (size,) = struct.unpack("<Q", head)
            data = f.read(size)
            if len(data) < size:
                raise ValueError(f"{path}: truncated entry")
            e = recording_pb2.RecordingEntry()
            e.ParseFromString(data)
            yield e


class PbscanWriter:
    """msensor `.pbscan`: repeated [uint64 LE size][RecordingEntry bytes] (see ScanRecorder::append)."""

    def __init__(self, path: str | Path) -> None:
        self._f = open(path, "wb")

    def _write(self, entry: "recording_pb2.RecordingEntry") -> None:
        data = entry.SerializeToString()
        self._f.write(struct.pack("<Q", len(data)))
        self._f.write(data)

    def on_imu(self, s: ImuSample) -> None:
        e = recording_pb2.RecordingEntry()
        e.imu.CopyFrom(imu_to_proto(s))
        self._write(e)

    def on_scan(self, scan: ScanData, gt: GroundTruth) -> None:
        e = recording_pb2.RecordingEntry()
        e.scan.CopyFrom(scan_to_proto(scan))
        self._write(e)

    def close(self) -> None:
        self._f.close()
