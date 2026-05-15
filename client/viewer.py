import logging
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import grpc
import numpy as np

from geometry import euler_xyz_to_wxyz
from points import intensity_to_colors, to_viser_colors, to_viser_points

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import lidar_pb2, slam_pb2, slam_pb2_grpc  # noqa: E402


logger = logging.getLogger(__name__)

DEFAULT_MAIN_LOOP_INTERVAL_SEC = 0.1
RECONNECT_INITIAL_DELAY_SEC = 1.0
RECONNECT_MAX_DELAY_SEC = 30.0
RECONNECT_BACKOFF_FACTOR = 2.0
MAP_POINT_SIZE_SCALE = 0.2
SCAN_POINT_SIZE_SCALE = 0.2
CORRESPONDENCE_POINT_SIZE_SCALE = 0.2
SCAN_COLOR = np.array([0, 255, 0], dtype=np.uint8)
CORRESPONDENCE_COLOR = np.array([255, 80, 80], dtype=np.uint8)


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
        self._map_lock = threading.Lock()
        self._viser: Any = viser
        self._map_point_chunks: list[np.ndarray] = []
        self._map_color_chunks: list[np.ndarray] = []
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
        map_increment_thread = threading.Thread(target=self._run_stream_map_increments, daemon=True)
        scan_thread = threading.Thread(target=self._run_stream_transformed_scan, daemon=True)
        correspondence_thread = threading.Thread(target=self._run_stream_correspondences, daemon=True)
        pose_thread = threading.Thread(target=self._run_stream_pose, daemon=True)
        map_increment_thread.start()
        scan_thread.start()
        correspondence_thread.start()
        pose_thread.start()

        try:
            while not self._shutdown_event.is_set():
                time.sleep(DEFAULT_MAIN_LOOP_INTERVAL_SEC)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
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
        delay = RECONNECT_INITIAL_DELAY_SEC
        while not self._shutdown_event.is_set():
            try:
                stream_fn()
                # Stream ended normally (server closed); reset backoff and reconnect.
                delay = RECONNECT_INITIAL_DELAY_SEC
            except Exception as exc:
                self._log_stream_exception(stream_name, exc)
                if self._shutdown_event.is_set():
                    return
                logger.info(
                    "Reconnecting %s stream in %.1f seconds...",
                    stream_name,
                    delay,
                )
                if self._shutdown_event.wait(timeout=delay):
                    return
                delay = min(delay * RECONNECT_BACKOFF_FACTOR, RECONNECT_MAX_DELAY_SEC)

    def _run_stream_map_increments(self) -> None:
        self._run_stream("map", self._stream_map_increments)

    def _run_stream_transformed_scan(self) -> None:
        self._run_stream("transformed scan", self._stream_transformed_scan)

    def _run_stream_correspondences(self) -> None:
        self._run_stream("correspondence", self._stream_correspondences)

    def _run_stream_pose(self) -> None:
        self._run_stream("pose", self._stream_pose)

    def _stream_map_increments(self) -> None:
        request = slam_pb2.Empty()

        with grpc.insecure_channel(self._server_addr) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)

            # Fetch full snapshot on each (re)connect so stale data is replaced.
            self._clear_local_clouds()
            logger.info("Fetching initial SLAM map snapshot from %s", self._server_addr)
            self._set_map_cloud(stub.GetMap(request))

            logger.info("Streaming map increments from %s", self._server_addr)
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
        self._scan_cloud.points = points
        self._scan_cloud.colors = np.tile(SCAN_COLOR, (len(points), 1))
        logger.info("Rendered transformed scan with %d points", len(points))

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
