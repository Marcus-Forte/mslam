import logging
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import grpc
import numpy as np

from connection import ConnectionMonitor, ConnectionState, led_html
from follow import FOLLOW_MODES, IDENTITY, FollowMode, FollowView, Transform
from geometry import euler_xyz_to_wxyz
from keyframe_renderer import KeyframeRenderer
from keyframe_store import KeyframeStore
from keyframe_stream import stream_keyframes, uses_pose_graph
from map_renderer import MapRenderer
from map_store import MapStore
from map_stream import ServerTooOldError, StreamEnd, stream_map
from points import to_viser_points
from sensor_status import GraphInfo, SensorState, SensorStatusMonitor, graph_led_html, sensor_led_html

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import lidar_pb2, slam_pb2, slam_pb2_grpc  # noqa: E402


logger = logging.getLogger(__name__)

DEFAULT_MAIN_LOOP_INTERVAL_SEC = 0.1
POSE_RENDER_INTERVAL_SEC = 1.0 / 30.0
"""The pose axes and the followed view are redrawn at this steady rate from the latest pose (poses arrive in
bursts, one per IMU sample, processed per scan), like rviz drawing the latest TF every frame."""
WORLD_NODE = "/world"
"""Every visual is a child of this frame; following moves it (see follow.py)."""
RECONNECT_DELAY_SEC = 5.0
RESYNC_DELAY_SEC = 1.0
KEYFRAME_STREAM_MAX_MESSAGE_BYTES = 64 * 1024 * 1024
"""StreamKeyframes sends one whole keyframe per message; the server allows 64 MiB, gRPC Python's default receive limit
is 4 MiB (about 250k points)."""
MAP_POINT_SIZE_SCALE = 0.2
SCAN_POINT_SIZE_SCALE = 0.2
CORRESPONDENCE_POINT_SIZE_SCALE = 0.2
SCAN_COLOR = np.array([0, 255, 0], dtype=np.uint8)
CORRESPONDENCE_COLOR = np.array([255, 80, 80], dtype=np.uint8)
RESET_CAMERA_LABEL = "Reset camera (R)"
RESET_CAMERA_OFFSET = np.array([1.25, -1.25, 0.95]) * 2.0
"""Where the reset view puts the camera relative to the origin (z up, looking at the origin)."""
RESET_CAMERA_KEY_HTML = (
    f'<img src="data:," style="display:none" onerror="'
    "if(!window.__slamResetKey){window.__slamResetKey=1;"
    "window.addEventListener('keydown',function(e){"
    "if(e.key!=='r'&&e.key!=='R')return;if(e.ctrlKey||e.metaKey||e.altKey||e.repeat)return;"
    "var t=document.activeElement;if(t&&/^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName))return;"
    "var b=Array.from(document.querySelectorAll('button')).find(function(x){"
    f"return x.textContent.trim()==='{RESET_CAMERA_LABEL}'}});"
    'if(b){e.preventDefault();b.click();}});}">'
)
"""viser has no keyboard API: this hidden element installs one page-level `r` listener that clicks the reset button."""


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
        self._viser: Any = viser
        self._server_too_old_logged = False
        self._viser_server = viser.ViserServer(host="0.0.0.0", port=viser_port)
        self._viser_server.scene.set_background_image(np.zeros((1, 1, 3), dtype=np.uint8))
        self._world_frame = self._viser_server.scene.add_frame(WORLD_NODE, show_axes=False)
        self._map_store = MapStore()
        self._map_renderer = MapRenderer(
            self._map_store,
            self._viser_server.scene,
            point_size=point_size * MAP_POINT_SIZE_SCALE,
            atomic=self._viser_server.atomic,
            node_prefix=f"{WORLD_NODE}/slam/map",
        )
        # Pose-graph strategy: one node per keyframe, placed by its pose (viewer.py picks this path from
        # GetStatus.strategy; the map renderer above stays unused then).
        self._keyframe_store = KeyframeStore()
        self._keyframe_renderer = KeyframeRenderer(
            self._keyframe_store,
            self._viser_server.scene,
            point_size=point_size * MAP_POINT_SIZE_SCALE,
            atomic=self._viser_server.atomic,
            node_prefix=f"{WORLD_NODE}/slam/keyframes",
        )
        self._map_mode: str | None = None
        self._scan_cloud = self._viser_server.scene.add_point_cloud(
            name=f"{WORLD_NODE}/slam/scan",
            points=np.empty((0, 3), dtype=np.float32),
            colors=np.empty((0, 3), dtype=np.uint8),
            point_size=point_size * SCAN_POINT_SIZE_SCALE,
            point_shape="rounded",
        )
        self._correspondence_cloud = self._viser_server.scene.add_point_cloud(
            name=f"{WORLD_NODE}/slam/correspondences",
            points=np.empty((0, 3), dtype=np.float32),
            colors=np.empty((0, 3), dtype=np.uint8),
            point_size=point_size * CORRESPONDENCE_POINT_SIZE_SCALE,
            point_shape="sparkle",
        )
        self._pose_frame = self._viser_server.scene.add_frame(
            name=f"{WORLD_NODE}/slam/pose",
            axes_length=0.25,
            axes_radius=0.01,
            origin_radius=0.02,
            position=(0.0, 0.0, 0.0),
            wxyz=(1.0, 0.0, 0.0, 0.0),
        )

        self._grpc_channel = grpc.insecure_channel(self._server_addr)
        self._control_stub = slam_pb2_grpc.SlamServiceStub(self._grpc_channel)

        self._connection_led = self._viser_server.gui.add_html(led_html(ConnectionState.CONNECTING, server_addr))
        self._connection_monitor = ConnectionMonitor(server_addr, self._show_connection)
        # The SLAM server's own link to its sensor server (the LED above is the viewer's link to the SLAM server).
        self._sensor_led = self._viser_server.gui.add_html(sensor_led_html(SensorState.UNKNOWN, ""))
        self._graph_led = self._viser_server.gui.add_html("")
        self._sensor_monitor = SensorStatusMonitor(self._control_stub, self._show_sensor, on_graph=self._show_graph)

        self._follow_dropdown = self._viser_server.gui.add_dropdown(
            "Follow",
            FOLLOW_MODES,
            initial_value=FollowMode.OFF.value,
            hint="Keep the sensor still on screen and move the world around it, as rviz does with a target frame. "
            "Position: rviz Orbit; Position + yaw: rviz ThirdPersonFollower; Full pose: also roll and pitch.",
        )
        # Pose-graph path only: shown when the viewer streams keyframes (see _choose_map_mode).
        self._axes_checkbox = self._viser_server.gui.add_checkbox(
            "Keyframe axes",
            initial_value=True,
            visible=False,
            hint="Draw a small coordinate axis at each pose-graph keyframe.",
        )

        @self._axes_checkbox.on_update
        def _on_axes_toggled(_: Any) -> None:
            self._keyframe_renderer.set_axes_visible(self._axes_checkbox.value)

        self._follow_view = FollowView()
        # Written by the pose stream thread, read by the pose render thread.
        self._pose_lock = threading.Lock()
        self._latest_pose: Transform = IDENTITY
        self._pose_version = 0

        start_button = self._viser_server.gui.add_button("Start")
        stop_button = self._viser_server.gui.add_button("Stop")
        reset_button = self._viser_server.gui.add_button("Reset")
        reset_camera_button = self._viser_server.gui.add_button(
            RESET_CAMERA_LABEL, hint="Look at the origin from the default viewpoint (shortcut: R)."
        )
        self._viser_server.gui.add_html(RESET_CAMERA_KEY_HTML)

        @reset_camera_button.on_click
        def _on_reset_camera(_: Any) -> None:
            self._reset_camera()

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
                # The map clears when the stream announces the new epoch (in every viewer, not only this one).
                self._clear_scan_visuals()
                logger.info("Sent Reset command")
            except grpc.RpcError as exc:
                logger.error("Reset RPC failed: %s", exc)

    def run(self) -> None:
        self._connection_monitor.start(self._shutdown_event.is_set, self._shutdown_event.wait)
        self._sensor_monitor.start(self._shutdown_event.is_set, self._shutdown_event.wait)
        map_increment_thread = threading.Thread(target=self._run_stream_map_increments, daemon=True)
        map_flush_thread = threading.Thread(
            target=self._map_renderer.run, args=(self._shutdown_event.is_set, self._shutdown_event.wait), daemon=True
        )
        keyframe_flush_thread = threading.Thread(
            target=self._keyframe_renderer.run,
            args=(self._shutdown_event.is_set, self._shutdown_event.wait),
            daemon=True,
        )
        scan_thread = threading.Thread(target=self._run_stream_transformed_scan, daemon=True)
        correspondence_thread = threading.Thread(target=self._run_stream_correspondences, daemon=True)
        pose_thread = threading.Thread(target=self._run_stream_pose, daemon=True)
        pose_render_thread = threading.Thread(target=self._run_pose_render, daemon=True)
        map_increment_thread.start()
        map_flush_thread.start()
        keyframe_flush_thread.start()
        scan_thread.start()
        correspondence_thread.start()
        pose_thread.start()
        pose_render_thread.start()

        try:
            while not self._shutdown_event.is_set():
                time.sleep(DEFAULT_MAIN_LOOP_INTERVAL_SEC)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            self._shutdown_event.set()
        self._connection_monitor.close()

    def _show_connection(self, state: ConnectionState) -> None:
        self._connection_led.content = led_html(state, self._server_addr)

    def _show_sensor(self, state: SensorState, address: str) -> None:
        self._sensor_led.content = sensor_led_html(state, address)

    def _show_graph(self, info: GraphInfo) -> None:
        self._graph_led.content = graph_led_html(info)

    def _log_stream_exception(self, stream_name: str, exc: Exception) -> None:
        if isinstance(exc, ServerTooOldError):
            # One clear error; the retry loop keeps trying at the normal interval, quietly.
            if not self._server_too_old_logged:
                self._server_too_old_logged = True
                logger.error(
                    "The SLAM server at %s does not implement StreamMap: it is too old for this viewer."
                    " Update the server; retrying every %.1f seconds.",
                    self._server_addr,
                    RECONNECT_DELAY_SEC,
                )
            return

        if isinstance(exc, grpc.RpcError):
            logger.error(
                "SLAM %s stream error: %s - %s",
                stream_name,
                exc.code().name,
                exc.details(),
            )
            return

        logger.exception("SLAM %s stream crashed", stream_name)

    def _clear_scan_visuals(self) -> None:
        empty_points = np.empty((0, 3), dtype=np.float32)
        empty_colors = np.empty((0, 3), dtype=np.uint8)

        self._scan_cloud.points = empty_points
        self._scan_cloud.colors = empty_colors

        self._correspondence_cloud.points = empty_points
        self._correspondence_cloud.colors = empty_colors

        self._set_latest_pose(IDENTITY)

    def _run_stream(
        self,
        stream_name: str,
        stream_fn: Callable[[], None],
    ) -> None:
        while not self._shutdown_event.is_set():
            try:
                stream_fn()
            except Exception as exc:
                self._log_stream_exception(stream_name, exc)
                if self._shutdown_event.is_set():
                    return
                logger.info(
                    "Reconnecting %s stream in %.1f seconds...",
                    stream_name,
                    RECONNECT_DELAY_SEC,
                )
                if self._shutdown_event.wait(timeout=RECONNECT_DELAY_SEC):
                    return

    def _run_stream_map_increments(self) -> None:
        self._run_stream("map", self._stream_map_increments)

    def _run_stream_transformed_scan(self) -> None:
        self._run_stream("transformed scan", self._stream_transformed_scan)

    def _run_stream_correspondences(self) -> None:
        self._run_stream("correspondence", self._stream_correspondences)

    def _run_stream_pose(self) -> None:
        self._run_stream("pose", self._stream_pose)

    def _choose_map_mode(self, stub: Any) -> str:
        """ "keyframes" when the server runs the pose-graph strategy, else "map". Decided once: the other path's
        store and scene nodes would otherwise be left behind, so a server restarted with another strategy needs the
        viewer restarted (logged)."""
        wanted = "keyframes" if uses_pose_graph(stub) else "map"
        if self._map_mode is None:
            self._map_mode = wanted
            self._axes_checkbox.visible = wanted == "keyframes"
            logger.info("Map stream: %s", "StreamKeyframes (pose graph)" if wanted == "keyframes" else "StreamMap")
        elif wanted != self._map_mode:
            logger.error(
                "The SLAM server now runs another strategy (%s); restart the viewer. Keeping %s.",
                wanted,
                self._map_mode,
            )
        return self._map_mode

    def _stream_map_increments(self) -> None:
        options = [("grpc.max_receive_message_length", KEYFRAME_STREAM_MAX_MESSAGE_BYTES)]
        with grpc.insecure_channel(self._server_addr, options=options) as channel:
            stub = slam_pb2_grpc.SlamServiceStub(channel)
            if self._choose_map_mode(stub) == "keyframes":
                # The open keyframe (the scans since the last one) is not in this stream: the live transformed
                # scan, drawn by the scan stream in the map frame, covers it.
                end = stream_keyframes(
                    stub,
                    self._keyframe_store,
                    on_change=lambda change: self._keyframe_renderer.notify() if change.changed else None,
                    should_stop=self._shutdown_event.is_set,
                )
                if end is StreamEnd.RESYNC:
                    self._shutdown_event.wait(timeout=RESYNC_DELAY_SEC)
                return
            # The local map stays on screen while disconnected; the request resumes from the stored position and
            # the map only clears if the server announces another epoch.
            end = stream_map(
                stub,
                self._map_store,
                on_change=lambda change: self._map_renderer.notify() if change.changed else None,
                should_stop=self._shutdown_event.is_set,
            )
        if end is StreamEnd.RESYNC:
            self._shutdown_event.wait(timeout=RESYNC_DELAY_SEC)

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

    def _update_scan_cloud(self, scan_snapshot: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(scan_snapshot)
        self._scan_cloud.points = points
        self._scan_cloud.colors = np.tile(SCAN_COLOR, (len(points), 1))
        logger.debug("Rendered transformed scan with %d points", len(points))

    def _update_correspondence_cloud(self, correspondences: lidar_pb2.PointCloud3) -> None:
        points = to_viser_points(correspondences)
        self._correspondence_cloud.points = points
        self._correspondence_cloud.colors = np.tile(CORRESPONDENCE_COLOR, (len(points), 1))
        logger.debug("Rendered correspondence snapshot with %d points", len(points))

    def _update_pose(self, pose: slam_pb2.Pose3D) -> None:
        rotation_wxyz = euler_xyz_to_wxyz(pose.phi, pose.omega, pose.theta).astype(np.float64)
        rotation_wxyz /= np.linalg.norm(rotation_wxyz)
        self._set_latest_pose((rotation_wxyz, np.array([pose.x, pose.y, pose.z], dtype=np.float64)))

    def _set_latest_pose(self, pose: Transform) -> None:
        with self._pose_lock:
            self._latest_pose = pose
            self._pose_version += 1

    def _run_pose_render(self) -> None:
        """Draw the latest pose and the followed view at a steady rate (only this thread touches them)."""
        rendered_version = -1
        mode = FollowMode.OFF
        while not self._shutdown_event.wait(timeout=POSE_RENDER_INTERVAL_SEC):
            try:
                with self._pose_lock:
                    pose, version = self._latest_pose, self._pose_version
                new_mode = FollowMode(self._follow_dropdown.value)
                if new_mode is not mode:
                    started = mode is FollowMode.OFF
                    self._follow_view.set_mode(new_mode, pose)
                    mode = new_mode
                    logger.info("Follow: %s", mode.value)
                    if started:
                        self._orbit_around(self._follow_view.on_screen(pose[1]))
                elif version == rendered_version:
                    continue
                rendered_version = version
                root_wxyz, root_position = self._follow_view.update(pose)
                # One unit per client: the root and the axes move in the same frame, so the axes never jitter
                # against the world.
                with self._viser_server.atomic():
                    self._world_frame.wxyz = root_wxyz
                    self._world_frame.position = root_position
                    self._pose_frame.wxyz = pose[0]
                    self._pose_frame.position = pose[1]
            except Exception:
                logger.exception("Pose rendering failed")

    def _reset_camera(self) -> None:
        """Put every client's camera at the default viewpoint, looking at the origin (as drawn, so it also works
        while following)."""
        target = self._follow_view.on_screen(np.zeros(3))
        for client in self._viser_server.get_clients().values():
            try:
                client.camera.up_direction = (0.0, 0.0, 1.0)
                client.camera.position = target + RESET_CAMERA_OFFSET
                client.camera.look_at = target
            except AssertionError:
                logger.debug("Client camera state not known yet; not resetting it")

    def _orbit_around(self, point: np.ndarray) -> None:
        """Make every client's camera orbit around `point` (rviz moves its focal point to the target frame)."""
        for client in self._viser_server.get_clients().values():
            try:
                client.camera.look_at = point
            except AssertionError:
                logger.debug("Client camera state not known yet; not moving its focal point")
