import argparse
import logging
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import grpc
import numpy as np

from colormap import rainbow_colormap

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import lidar_pb2, slam_pb2, slam_pb2_grpc


logger = logging.getLogger(__name__)

DEFAULT_SERVER_ADDR = "127.0.0.1:50052"
DEFAULT_VISER_PORT = 8080
DEFAULT_POINT_SIZE = 0.03
DEFAULT_MAIN_LOOP_INTERVAL_SEC = 0.1
MAP_POINT_SIZE_SCALE = 0.5
SCAN_POINT_SIZE_SCALE = 0.2
CORRESPONDENCE_POINT_SIZE_SCALE = 0.2
MAX_ACCUMULATED_SCAN_POINTS = 500_000
SCAN_COLOR = np.array([0, 255, 0], dtype=np.uint8)
TRANSFORMED_SCAN_COLOR = np.array([180, 180, 255], dtype=np.uint8)
CORRESPONDENCE_COLOR = np.array([255, 80, 80], dtype=np.uint8)


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
        self._map_point_chunks: list[np.ndarray] = []
        self._map_color_chunks: list[np.ndarray] = []
        self._accumulated_scan_chunks: list[np.ndarray] = []
        self._accumulated_scan_color_chunks: list[np.ndarray] = []
        self._accumulated_scan_count = 0
        self._viser_server = viser.ViserServer(port=viser_port)
        self._viser_server.scene.set_background_image(np.zeros((1, 1, 3), dtype=np.uint8))
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

        self._grpc_channel = grpc.insecure_channel(self._server_addr)
        self._control_stub = slam_pb2_grpc.SlamServiceStub(self._grpc_channel)

        start_button = self._viser_server.gui.add_button("Start")
        stop_button = self._viser_server.gui.add_button("Stop")
        reset_button = self._viser_server.gui.add_button("Reset")

        @start_button.on_click
        def _on_start(_: Any) -> None:
            try:
                self._control_stub.Start(slam_pb2.Empty())
                logger.info("Sent Start command")
            except grpc.RpcError as exc:
                logger.error("Start RPC failed: %s", exc)

        @stop_button.on_click
        def _on_stop(_: Any) -> None:
            try:
                self._control_stub.Stop(slam_pb2.Empty())
                logger.info("Sent Stop command")
            except grpc.RpcError as exc:
                logger.error("Stop RPC failed: %s", exc)

        @reset_button.on_click
        def _on_reset(_: Any) -> None:
            try:
                self._control_stub.Reset(slam_pb2.Empty())
                self._clear_local_clouds()
                logger.info("Sent Reset command")
            except grpc.RpcError as exc:
                logger.error("Reset RPC failed: %s", exc)

    def run(self) -> None:
        map_thread = threading.Thread(target=self._run_stream_map, daemon=True)
        map_increment_thread = threading.Thread(target=self._run_stream_map_increments, daemon=True)
        scan_thread = threading.Thread(target=self._run_stream_transformed_scan, daemon=True)
        correspondence_thread = threading.Thread(target=self._run_stream_correspondences, daemon=True)
        pose_thread = threading.Thread(target=self._run_stream_pose, daemon=True)
        map_thread.start()
        map_increment_thread.start()
        # scan_thread.start()
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
            return f"SLAM stream failed: {exc.code().name} - {exc.details()}"

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

    def _clear_local_clouds(self) -> None:
        empty_points = np.empty((0, 3), dtype=np.float32)
        empty_colors = np.empty((0, 3), dtype=np.uint8)

        with self._map_lock:
            self._map_point_chunks.clear()
            self._map_color_chunks.clear()
        self._cloud.points = empty_points
        self._cloud.colors = empty_colors

        with self._transformed_scan_lock:
            self._accumulated_scan_chunks.clear()
            self._accumulated_scan_color_chunks.clear()
            self._accumulated_scan_count = 0
        self._accumulated_scan_cloud.points = empty_points
        self._accumulated_scan_cloud.colors = empty_colors

        self._scan_cloud.points = empty_points
        self._scan_cloud.colors = empty_colors

        self._correspondence_cloud.points = empty_points
        self._correspondence_cloud.colors = empty_colors

        self._pose_frame.position = (0.0, 0.0, 0.0)
        self._pose_frame.wxyz = (1.0, 0.0, 0.0, 0.0)

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
        points = to_viser_points(map_snapshot)
        colors = intensity_to_colors(map_snapshot, len(points))
        if colors is None:
            colors = to_viser_colors(points)

        with self._map_lock:
            self._map_point_chunks = [points]
            self._map_color_chunks = [colors]

        self._cloud.points = points
        self._cloud.colors = colors
        logger.info("Rendered initial map snapshot with %d points", len(points))

    def _append_map_increment(self, increment: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(increment)
        colors = intensity_to_colors(increment, len(points))
        if colors is None:
            colors = to_viser_colors(points)

        with self._map_lock:
            self._map_point_chunks.append(points)
            self._map_color_chunks.append(colors)
            map_points = np.concatenate(self._map_point_chunks, axis=0)
            map_colors = np.concatenate(self._map_color_chunks, axis=0)

        self._cloud.points = map_points
        self._cloud.colors = map_colors
        logger.info(
            "Appended %d map increment points; map now has %d points",
            len(points),
            len(map_points),
        )

    def _update_scan_cloud(self, scan_snapshot: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(scan_snapshot)
        colors = intensity_to_colors(scan_snapshot, len(points))
        if colors is None:
            colors = to_viser_colors(points)

        self._scan_cloud.points = points
        self._scan_cloud.colors = np.tile(SCAN_COLOR, (len(points), 1))

        with self._transformed_scan_lock:
            self._accumulated_scan_chunks.append(points)
            self._accumulated_scan_color_chunks.append(colors)
            self._accumulated_scan_count += len(points)

            # Trim oldest chunks when over budget
            while self._accumulated_scan_count > MAX_ACCUMULATED_SCAN_POINTS and len(self._accumulated_scan_chunks) > 1:
                removed = self._accumulated_scan_chunks.pop(0)
                self._accumulated_scan_color_chunks.pop(0)
                self._accumulated_scan_count -= len(removed)

            acc_points = np.concatenate(self._accumulated_scan_chunks, axis=0)
            acc_colors = np.concatenate(self._accumulated_scan_color_chunks, axis=0)

        self._accumulated_scan_cloud.points = acc_points
        self._accumulated_scan_cloud.colors = acc_colors
        logger.info(
            "Rendered transformed scan with %d points; accumulated scans now %d points",
            len(points),
            len(acc_points),
        )

    def _update_correspondence_cloud(self, correspondences: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(correspondences)
        self._correspondence_cloud.points = points
        self._correspondence_cloud.colors = np.tile(CORRESPONDENCE_COLOR, (len(points), 1))
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
