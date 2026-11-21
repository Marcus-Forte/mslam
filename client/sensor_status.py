"""Sensor LED: whether the SLAM server is connected to its sensor server.

The SLAM server answers the unary `GetStatus` RPC with the state of its own link to the sensor server; the monitor
polls it about once a second. This is a different link from the one `connection.py` shows (viewer to SLAM server).

States:

* green, streaming: connected, and both LiDAR and IMU samples are arriving.
* amber, connected, no data: connected, but samples are not arriving on both streams (a paused playback server, an
  idle sensor).
* red, disconnected: the sensor server is unreachable.
* grey, on-board sensor: the SLAM server reads a local sensor (`remote_scanner` is "local"); there is no link.
* grey, unknown: the SLAM server cannot say, or the viewer cannot ask: it is unreachable (see the connection LED) or
  too old to implement `GetStatus`. The viewer keeps working with an old server, just without this LED.
"""

import html
import logging
from dataclasses import dataclass
import sys
import threading
from collections.abc import Callable
from enum import Enum
from pathlib import Path
from typing import Any

import grpc

from connection import led_row_html

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import slam_pb2  # noqa: E402


logger = logging.getLogger(__name__)

POLL_INTERVAL_SEC = 1.0
RPC_TIMEOUT_SEC = 2.0
"""A status call that takes longer than this counts as the SLAM server being unreachable."""

GREY = "#868e96"


class SensorState(Enum):
    STREAMING = ("streaming", "#2fb344")
    IDLE = ("connected, no data", "#f59f00")
    DISCONNECTED = ("disconnected", "#e03131")
    NOT_APPLICABLE = ("on-board sensor", GREY)
    UNKNOWN = ("unknown", GREY)
    SERVER_UNREACHABLE = ("unknown (SLAM server unreachable)", GREY)
    SERVER_TOO_OLD = ("unknown (SLAM server too old)", GREY)

    @property
    def label(self) -> str:
        return self.value[0]

    @property
    def color(self) -> str:
        return self.value[1]


_LINK_TO_STATE = {
    slam_pb2.SENSOR_LINK_STREAMING: SensorState.STREAMING,
    slam_pb2.SENSOR_LINK_CONNECTED_IDLE: SensorState.IDLE,
    slam_pb2.SENSOR_LINK_DISCONNECTED: SensorState.DISCONNECTED,
    slam_pb2.SENSOR_LINK_NOT_APPLICABLE: SensorState.NOT_APPLICABLE,
}


def state_from_status(status: Any) -> SensorState:
    """The LED state for a `SlamStatus` message; a link value this viewer does not know is UNKNOWN."""
    return _LINK_TO_STATE.get(status.sensor_link, SensorState.UNKNOWN)


@dataclass(frozen=True)
class GraphInfo:
    """The pose-graph part of `SlamStatus`. `pose_graph` is False for the odometry strategy (and for a server too
    old to say), in which case the counts are 0."""

    pose_graph: bool = False
    keyframes: int = 0
    loop_closures: int = 0


def graph_info_from_status(status: Any) -> GraphInfo:
    if status.strategy != slam_pb2.SLAM_STRATEGY_POSE_GRAPH:
        return GraphInfo()
    return GraphInfo(True, status.keyframes, status.loop_closures)


def graph_led_html(info: GraphInfo) -> str:
    """The status row for the pose graph; an empty string for odometry (nothing to show)."""
    if not info.pose_graph:
        return ""
    closures = f"{info.loop_closures} loop closure{'s' if info.loop_closures != 1 else ''}"
    return led_row_html("#228be6", f"Pose graph: {info.keyframes} keyframes, {closures}")


def sensor_led_html(state: SensorState, address: str) -> str:
    prefix = f"Sensor <code>{html.escape(address)}</code>" if address else "Sensor"
    return led_row_html(state.color, f"{prefix}: {state.label}")


class SensorStatusMonitor:
    """Polls `GetStatus` and calls `on_change(state, address)` whenever either changes (from its own thread), and once
    at start."""

    def __init__(
        self,
        stub: Any,
        on_change: Callable[[SensorState, str], None],
        poll_interval: float = POLL_INTERVAL_SEC,
        rpc_timeout: float = RPC_TIMEOUT_SEC,
        on_graph: Callable[[GraphInfo], None] | None = None,
    ) -> None:
        """`on_graph(info)` (optional) is called from the polling thread when the pose-graph part of the status
        changes, and once at start with the default (odometry) info."""
        self._stub = stub
        self._on_change = on_change
        self._on_graph = on_graph
        self._graph = GraphInfo()
        self._latest_graph: GraphInfo | None = None
        self._poll_interval = poll_interval
        self._rpc_timeout = rpc_timeout
        self._state = SensorState.UNKNOWN
        self._address = ""
        self._thread: threading.Thread | None = None

    @property
    def state(self) -> SensorState:
        return self._state

    def poll_once(self) -> tuple[SensorState, str]:
        try:
            status = self._stub.GetStatus(slam_pb2.Empty(), timeout=self._rpc_timeout)
        except grpc.RpcError as exc:
            if exc.code() == grpc.StatusCode.UNIMPLEMENTED:
                return SensorState.SERVER_TOO_OLD, ""
            return SensorState.SERVER_UNREACHABLE, ""
        self._latest_graph = graph_info_from_status(status)
        return state_from_status(status), status.sensor_address

    def start(self, should_stop: Callable[[], bool], wait: Callable[[float], bool]) -> None:
        """Start polling; `wait(seconds)` sleeps and returns True when the viewer shuts down."""
        self._on_change(self._state, self._address)
        if self._on_graph is not None:
            self._on_graph(self._graph)
        self._thread = threading.Thread(target=self._loop, args=(should_stop, wait), daemon=True)
        self._thread.start()

    def _loop(self, should_stop: Callable[[], bool], wait: Callable[[float], bool]) -> None:
        while not should_stop():
            state, address = self.poll_once()
            if (state, address) != (self._state, self._address):
                self._log(state, address)
                self._state, self._address = state, address
                self._on_change(state, address)
            if self._on_graph is not None and self._latest_graph not in (None, self._graph):
                self._graph = self._latest_graph
                self._on_graph(self._graph)
            if wait(self._poll_interval):
                return

    @staticmethod
    def _log(state: SensorState, address: str) -> None:
        where = f" {address}" if address else ""
        if state is SensorState.DISCONNECTED:
            logger.warning("Sensor server%s: %s", where, state.label)
        elif state in (SensorState.SERVER_UNREACHABLE, SensorState.UNKNOWN):
            # The connection LED and its log line already cover an unreachable SLAM server.
            logger.debug("Sensor status: %s", state.label)
        elif state is SensorState.SERVER_TOO_OLD:
            logger.warning("The SLAM server does not implement GetStatus: the sensor LED stays unknown.")
        else:
            logger.info("Sensor server%s: %s", where, state.label)
