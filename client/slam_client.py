import argparse
import logging
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any
from typing import Sequence

import grpc
import numpy as np

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import lidar_pb2, slam_pb2, slam_pb2_grpc


logger = logging.getLogger(__name__)

DEFAULT_SERVER_ADDR = "127.0.0.1:50052"
DEFAULT_VISER_PORT = 8080
DEFAULT_POINT_SIZE = 0.03
DEFAULT_MAIN_LOOP_INTERVAL_SEC = 0.1
MAP_POINT_SIZE_SCALE = 0.1
SCAN_POINT_SIZE_SCALE = 0.2
CORRESPONDENCE_POINT_SIZE_SCALE = 0.2
SCAN_COLOR = np.array([0, 255, 0], dtype=np.uint8)
TRANSFORMED_SCAN_COLOR = np.array([180, 180, 255], dtype=np.uint8)
CORRESPONDENCE_COLOR = np.array([255, 80, 80], dtype=np.uint8)


def to_viser_points(points: Sequence[lidar_pb2.Point3]) -> np.ndarray:
    arr = np.empty((len(points), 3), dtype=np.float32)
    for i, point in enumerate(points):
        arr[i, 0] = point.x
        arr[i, 1] = point.y
        arr[i, 2] = point.z
    return arr


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


class SlamViewerClient:
    def __init__(self, server_addr: str, viser_port: int, point_size: float) -> None:
        try:
            import viser
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "viser is not installed in this environment. Install client dependencies first."
            ) from exc

        self._server_addr = server_addr
        self._shutdown_event = threading.Event()
        self._fatal_exception: BaseException | None = None
        self._fatal_exception_lock = threading.Lock()
        self._map_lock = threading.Lock()
        self._transformed_scan_lock = threading.Lock()
        self._viser: Any = viser
        self._map_points = np.empty((0, 3), dtype=np.float32)
        self._map_colors = np.empty((0, 3), dtype=np.uint8)
        self._accumulated_scan_points = np.empty((0, 3), dtype=np.float32)
        self._accumulated_scan_colors = np.empty((0, 3), dtype=np.uint8)
        self._viser_server = viser.ViserServer(port=viser_port)
        self._viser_server.scene.set_background_image(
            np.zeros((1, 1, 3), dtype=np.uint8)
        )
        self._cloud = self._viser_server.scene.add_point_cloud(
            name="/slam/map",
            points=np.empty((0, 3), dtype=np.float32),
            colors=np.empty((0, 3), dtype=np.uint8),
            point_size=point_size * MAP_POINT_SIZE_SCALE,
            point_shape="rounded",
        )
        self._scan_cloud = self._viser_server.scene.add_point_cloud(
            name="/slam/scan",
            points=np.empty((0, 3), dtype=np.float32),
            colors=np.empty((0, 3), dtype=np.uint8),
            point_size=point_size * SCAN_POINT_SIZE_SCALE,
            point_shape="rounded",
        )
        self._accumulated_scan_cloud = self._viser_server.scene.add_point_cloud(
            name="/slam/accumulated_scans",
            points=np.empty((0, 3), dtype=np.float32),
            colors=np.empty((0, 3), dtype=np.uint8),
            point_size=point_size * SCAN_POINT_SIZE_SCALE,
            point_shape="rounded",
        )
        self._correspondence_cloud = self._viser_server.scene.add_point_cloud(
            name="/slam/correspondences",
            points=np.empty((0, 3), dtype=np.float32),
            colors=np.empty((0, 3), dtype=np.uint8),
            point_size=point_size * CORRESPONDENCE_POINT_SIZE_SCALE,
            point_shape="sparkle",
        )
        self._pose_frame = self._viser_server.scene.add_frame(
            name="/slam/pose",
            axes_length=0.25,
            axes_radius=0.01,
            origin_radius=0.02,
            position=(0.0, 0.0, 0.0),
            wxyz=(1.0, 0.0, 0.0, 0.0),
        )

    def run(self) -> None:
        map_thread = threading.Thread(target=self._run_stream_map, daemon=True)
        map_increment_thread = threading.Thread(
            target=self._run_stream_map_increments, daemon=True
        )
        scan_thread = threading.Thread(
            target=self._run_stream_transformed_scan, daemon=True
        )
        correspondence_thread = threading.Thread(
            target=self._run_stream_correspondences, daemon=True
        )
        pose_thread = threading.Thread(target=self._run_stream_pose, daemon=True)
        map_thread.start()
        map_increment_thread.start()
        scan_thread.start()
        correspondence_thread.start()
        pose_thread.start()

        while not self._shutdown_event.is_set():
            time.sleep(DEFAULT_MAIN_LOOP_INTERVAL_SEC)

        raise SystemExit(self._format_fatal_exception())

    def _format_fatal_exception(self) -> str:
        exc = self._fatal_exception
        if exc is None:
            return "SLAM viewer stopped unexpectedly."

        if isinstance(exc, grpc.RpcError):
            return (
                "SLAM stream failed: "
                f"{exc.code().name} - {exc.details()}"
            )

        return f"SLAM stream crashed: {exc}"

    def _set_fatal_exception(self, exc: BaseException) -> None:
        with self._fatal_exception_lock:
            if self._fatal_exception is None:
                self._fatal_exception = exc
                self._shutdown_event.set()

    def _log_stream_exception(self, stream_name: str, exc: Exception) -> None:
        if isinstance(exc, grpc.RpcError):
            logger.error(
                "SLAM %s stream error: %s - %s",
                stream_name,
                exc.code().name,
                exc.details(),
            )
            return

        logger.exception("SLAM %s stream crashed", stream_name)

    def _run_stream(
        self,
        stream_name: str,
        stream_fn: Callable[[], None],
    ) -> None:
        try:
            stream_fn()
        except Exception as exc:
            self._log_stream_exception(stream_name, exc)
            self._set_fatal_exception(exc)

    def _run_stream_map(self) -> None:
        self._run_stream("map", self._stream_map)

    def _run_stream_map_increments(self) -> None:
        self._run_stream("map increments", self._stream_map_increments)

    def _run_stream_transformed_scan(self) -> None:
        self._run_stream("transformed scan", self._stream_transformed_scan)

    def _run_stream_correspondences(self) -> None:
        self._run_stream("correspondence", self._stream_correspondences)

    def _run_stream_pose(self) -> None:
        self._run_stream("pose", self._stream_pose)

    def _stream_map(self) -> None:
        request = slam_pb2.Empty()

        logger.info("Fetching initial SLAM map snapshot from %s", self._server_addr)
        with grpc.insecure_channel(self._server_addr) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)
            self._set_map_cloud(stub.GetMap(request))

    def _stream_map_increments(self) -> None:
        request = slam_pb2.Empty()

        logger.info(
            "Connecting to SLAM map increments stream at %s",
            self._server_addr,
        )
        with grpc.insecure_channel(self._server_addr) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)
            for increment in stub.GetMapIncrements(request):
                if self._shutdown_event.is_set():
                    break
                self._append_map_increment(increment)

    def _stream_transformed_scan(self) -> None:
        request = slam_pb2.Empty()

        logger.info(
            "Connecting to SLAM transformed scan stream at %s",
            self._server_addr,
        )
        with grpc.insecure_channel(self._server_addr) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)
            for scan_snapshot in stub.GetTransformedScan(request):
                if self._shutdown_event.is_set():
                    break
                self._update_scan_cloud(scan_snapshot)

    def _stream_correspondences(self) -> None:
        request = slam_pb2.Empty()

        logger.info(
            "Connecting to SLAM correspondence stream at %s",
            self._server_addr,
        )
        with grpc.insecure_channel(self._server_addr) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)
            for correspondences in stub.GetCorrespondences(request):
                if self._shutdown_event.is_set():
                    break
                self._update_correspondence_cloud(correspondences)

    def _stream_pose(self) -> None:
        request = slam_pb2.Empty()

        logger.info("Connecting to SLAM pose stream at %s", self._server_addr)
        with grpc.insecure_channel(self._server_addr) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)
            for pose in stub.GetPose(request):
                if self._shutdown_event.is_set():
                    break
                self._update_pose(pose)

    def _set_map_cloud(self, map_snapshot: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(map_snapshot.points)
        colors = to_viser_colors(points)

        with self._map_lock:
            self._map_points = points
            self._map_colors = colors

        self._cloud.points = points
        self._cloud.colors = colors
        logger.info("Rendered initial map snapshot with %d points", len(points))

    def _append_map_increment(self, increment: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(increment.points)
        colors = to_viser_colors(points)

        with self._map_lock:
            self._map_points = np.concatenate((self._map_points, points), axis=0)
            self._map_colors = np.concatenate((self._map_colors, colors), axis=0)
            map_points = self._map_points
            map_colors = self._map_colors

        self._cloud.points = map_points
        self._cloud.colors = map_colors
        logger.info(
            "Appended %d map increment points; map now has %d points",
            len(points),
            len(map_points),
        )

    def _update_scan_cloud(self, scan_snapshot: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(scan_snapshot.points)
        colors = to_viser_colors(points)

        self._scan_cloud.points = points
        self._scan_cloud.colors = np.tile(SCAN_COLOR, (len(points), 1))

        with self._transformed_scan_lock:
            self._accumulated_scan_points = np.concatenate(
                (self._accumulated_scan_points, points), axis=0
            )
            self._accumulated_scan_colors = np.concatenate(
                (self._accumulated_scan_colors, colors), axis=0
            )
            acc_points = self._accumulated_scan_points
            acc_colors = self._accumulated_scan_colors

        self._accumulated_scan_cloud.points = acc_points
        self._accumulated_scan_cloud.colors = acc_colors
        logger.info(
            "Rendered transformed scan with %d points; accumulated scans now %d points",
            len(points),
            len(acc_points),
        )

    def _update_correspondence_cloud(
        self, correspondences: lidar_pb2.PointCloud3
    ) -> None:
        points = to_viser_points(correspondences.points)
        self._correspondence_cloud.points = points
        self._correspondence_cloud.colors = np.tile(
            CORRESPONDENCE_COLOR, (len(points), 1)
        )
        logger.info("Rendered correspondence snapshot with %d points", len(points))

    def _update_pose(self, pose: slam_pb2.Pose3D) -> None:
        position = np.array([pose.x, pose.y, pose.z], dtype=np.float32)
        rotation_wxyz = euler_xyz_to_wxyz(pose.phi, pose.omega, pose.theta)

        self._pose_frame.position = position
        self._pose_frame.wxyz = rotation_wxyz


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to the mslam gRPC server and render map snapshots and transformed scans in viser."
    )
    parser.add_argument(
        "--server-addr",
        default=DEFAULT_SERVER_ADDR,
        help="Address of the mslam gRPC server.",
    )
    parser.add_argument(
        "--viser-port",
        type=int,
        default=DEFAULT_VISER_PORT,
        help="Port for the local viser web UI.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=DEFAULT_POINT_SIZE,
        help="Rendered point size in viser.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    try:
        viewer = SlamViewerClient(
            server_addr=args.server_addr,
            viser_port=args.viser_port,
            point_size=args.point_size,
        )
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    viewer.run()


if __name__ == "__main__":
    main()