import argparse
import logging
import platform
import threading
import time
from typing import Any
from typing import Sequence

import grpc
import numpy as np

from proto_gen import lidar_pb2, slam_pb2, slam_pb2_grpc


logger = logging.getLogger(__name__)

DEFAULT_SERVER_ADDR = "127.0.0.1:50052"
DEFAULT_VISER_PORT = 8080
DEFAULT_POINT_SIZE = 0.03
DEFAULT_RECONNECT_DELAY_SEC = 1.0
DEFAULT_POSE_POLL_INTERVAL_SEC = 0.1
MAP_POINT_SIZE_SCALE = 0.2
SCAN_POINT_SIZE_SCALE = 0.4
CORRESPONDENCE_POINT_SIZE_SCALE = 0.55
SCAN_COLOR = np.array([0, 255, 0], dtype=np.uint8)
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
            machine = platform.machine().lower()
            system = platform.system().lower()
            if system == "linux" and machine == "aarch64":
                raise RuntimeError(
                    "viser is not installed for this client environment. On linux/aarch64, "
                    "the current viser dependency chain pulls in embreex, which has no compatible wheel."
                ) from exc

            raise RuntimeError(
                "viser is not installed in this environment. Install client dependencies first."
            ) from exc

        self._server_addr = server_addr
        self._viser: Any = viser
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
        map_thread = threading.Thread(target=self._stream_map, daemon=True)
        scan_thread = threading.Thread(target=self._stream_scan, daemon=True)
        correspondence_thread = threading.Thread(
            target=self._stream_correspondences, daemon=True
        )
        pose_thread = threading.Thread(target=self._stream_pose, daemon=True)
        map_thread.start()
        scan_thread.start()
        correspondence_thread.start()
        pose_thread.start()

        while True:
            time.sleep(DEFAULT_RECONNECT_DELAY_SEC)

    def _sleep_before_retry(self, stream_name: str) -> None:
        logger.info(
            "Retrying %s stream in %.1f seconds",
            stream_name,
            DEFAULT_RECONNECT_DELAY_SEC,
        )
        time.sleep(DEFAULT_RECONNECT_DELAY_SEC)

    def _log_stream_exception(self, stream_name: str, exc: Exception) -> None:
        if isinstance(exc, grpc.RpcError):
            logger.warning(
                "SLAM %s stream error: %s - %s",
                stream_name,
                exc.code().name,
                exc.details(),
            )
            return

        logger.exception("SLAM %s stream crashed", stream_name)

    def _stream_map(self) -> None:
        request = slam_pb2.Empty()

        while True:
            try:
                logger.info("Connecting to SLAM map stream at %s", self._server_addr)
                with grpc.insecure_channel(self._server_addr) as channel:
                    stub = slam_pb2_grpc.SlamServiceStub(channel)
                    for map_snapshot in stub.GetMap(request):
                        self._update_cloud(map_snapshot)
            except Exception as exc:
                self._log_stream_exception("map", exc)

            self._sleep_before_retry("map")

    def _stream_scan(self) -> None:
        request = slam_pb2.Empty()

        while True:
            try:
                logger.info("Connecting to SLAM scan stream at %s", self._server_addr)
                with grpc.insecure_channel(self._server_addr) as channel:
                    stub = slam_pb2_grpc.SlamServiceStub(channel)
                    for scan_snapshot in stub.GetScan(request):
                        self._update_scan_cloud(scan_snapshot)
            except Exception as exc:
                self._log_stream_exception("scan", exc)

            self._sleep_before_retry("scan")

    def _stream_correspondences(self) -> None:
        request = slam_pb2.Empty()

        while True:
            try:
                logger.info(
                    "Connecting to SLAM correspondence stream at %s",
                    self._server_addr,
                )
                with grpc.insecure_channel(self._server_addr) as channel:
                    stub = slam_pb2_grpc.SlamServiceStub(channel)
                    for correspondences in stub.GetCorrespondences(request):
                        self._update_correspondence_cloud(correspondences)
            except Exception as exc:
                self._log_stream_exception("correspondence", exc)

            self._sleep_before_retry("correspondence")

    def _stream_pose(self) -> None:
        request = slam_pb2.Empty()

        while True:
            try:
                logger.info("Connecting to SLAM pose stream at %s", self._server_addr)
                with grpc.insecure_channel(self._server_addr) as channel:
                    stub = slam_pb2_grpc.SlamServiceStub(channel)
                    while True:
                        pose = stub.GetPose(request)
                        self._update_pose(pose)
                        time.sleep(DEFAULT_POSE_POLL_INTERVAL_SEC)
            except Exception as exc:
                self._log_stream_exception("pose", exc)

            self._sleep_before_retry("pose")

    def _update_cloud(self, map_snapshot: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(map_snapshot.points)
        self._cloud.points = points
        self._cloud.colors = to_viser_colors(points)
        logger.info("Rendered map snapshot with %d points", len(points))

    def _update_scan_cloud(self, scan_snapshot: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(scan_snapshot.points)
        self._scan_cloud.points = points
        self._scan_cloud.colors = np.tile(SCAN_COLOR, (len(points), 1))
        logger.info("Rendered scan snapshot with %d points", len(points))

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
        self._pose_frame.position = np.array([pose.x, pose.y, pose.z], dtype=np.float32)
        self._pose_frame.wxyz = euler_xyz_to_wxyz(pose.phi, pose.omega, pose.theta)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to the mslam gRPC server and render the map in viser."
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