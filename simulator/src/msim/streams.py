"""The msensor gRPC service: LidarService and ImuService on one port.

Mirrors the real server (`third_party/msensor/grpc/{sensor_stream.hh,
lidar_service.cc,imu_service.cc,msensor_server.cc}`):

* every client stream has its own bounded FIFO. `max_queued_samples` 0 selects
  the stream default, a larger value is lowered to the cap (IMU default 200 /
  cap 4000, LiDAR default 1 / cap 64), never less than 1;
* when the FIFO is full the OLDEST sample is dropped and counted, so a slow
  client skips samples instead of blocking the sensor (`drop` mode, the real
  behaviour and the default for paced runs);
* at most 8 clients per stream (RESOURCE_EXHAUSTED beyond that); a stream whose
  sensor is disabled answers UNAVAILABLE;
* sample numbers are consecutive per stream; the IMU counts from the start of
  the simulation whether or not a client is connected (as the Mid360 IMU does)
  while a scan only gets a number if it was generated for a consumer.

Addition for deterministic runs: `block` mode makes the producer wait for room
instead of dropping, so an unpaced run loses nothing (the real server has no
such mode; a live sensor cannot be held back).
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from concurrent import futures
from dataclasses import dataclass
from typing import Any

import grpc

from ._proto import ensure_generated

ensure_generated()
import header_pb2  # noqa: E402,F401
import imu_pb2  # noqa: E402
import imu_pb2_grpc  # noqa: E402
import lidar_pb2  # noqa: E402
import lidar_pb2_grpc  # noqa: E402

from .engine import GroundTruth  # noqa: E402
from .sensors import ImuSample, ScanData  # noqa: E402

log = logging.getLogger("msim.streams")

MAX_CLIENTS_PER_STREAM = 8


@dataclass(frozen=True)
class QueueLimits:
    default: int
    cap: int


IMU_LIMITS = QueueLimits(200, 4000)
LIDAR_LIMITS = QueueLimits(1, 64)


def resolve_queue_size(requested: int, limits: QueueLimits) -> int:
    size = limits.default if requested == 0 else requested
    return max(1, min(size, limits.cap))


class _Closed:
    pass


CLOSED = _Closed()


class Subscription:
    def __init__(self, capacity: int, block: bool, owner: "Stream") -> None:
        self.capacity = capacity
        self.block = block
        self._owner = owner
        self._q: deque[Any] = deque()
        self._cv = threading.Condition()
        self._closed = False
        self.dropped = 0

    def put(self, item: Any, abort: threading.Event) -> None:
        with self._cv:
            if self.block:
                while len(self._q) >= self.capacity and not self._closed and not abort.is_set():
                    self._cv.wait(0.1)
                if self._closed or abort.is_set():
                    return
            self._q.append(item)
            if len(self._q) > self.capacity:
                self._q.popleft()
                self.dropped += 1
            self._cv.notify_all()

    def get(self, timeout: float) -> Any | None:
        """Oldest item; None on timeout; CLOSED once closed and drained."""
        with self._cv:
            if not self._q and not self._closed:
                self._cv.wait(timeout)
            if self._q:
                item = self._q.popleft()
                self._cv.notify_all()
                return item
            return CLOSED if self._closed else None

    def close(self) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        self._owner._remove(self)


class Stream:
    def __init__(self, name: str, limits: QueueLimits, available: bool, block: bool) -> None:
        self.name, self.limits, self.available, self.block = name, limits, available, block
        self._subs: list[Subscription] = []
        self._lock = threading.Lock()
        self._abort = threading.Event()
        self.published = 0

    def subscribe(self, requested: int) -> Subscription:
        with self._lock:
            if len(self._subs) >= MAX_CLIENTS_PER_STREAM:
                raise OverflowError(f"Too many clients on this stream (maximum {MAX_CLIENTS_PER_STREAM})")
            sub = Subscription(resolve_queue_size(requested, self.limits), self.block, self)
            self._subs.append(sub)
        log.info("Start %s stream (queue %d).", self.name, sub.capacity)
        return sub

    def _remove(self, sub: Subscription) -> None:
        with self._lock:
            if sub in self._subs:
                self._subs.remove(sub)
                log.info("Ending %s stream (dropped %d samples).", self.name, sub.dropped)

    def subscribers(self) -> int:
        with self._lock:
            return len(self._subs)

    def publish(self, item: Any) -> None:
        with self._lock:
            subs = list(self._subs)
        self.published += 1
        for s in subs:
            s.put(item, self._abort)

    def close_all(self) -> None:
        self._abort.set()
        with self._lock:
            subs = list(self._subs)
        for s in subs:
            with s._cv:
                s._closed = True
                s._cv.notify_all()


def _fill_scan(msg: "lidar_pb2.PointCloud3", scan: ScanData) -> None:
    msg.header.timestamp = scan.t0_ns
    msg.header.sequence_number = scan.seq & 0xFFFFFFFF
    msg.x.extend(scan.xyz[:, 0].tolist())
    msg.y.extend(scan.xyz[:, 1].tolist())
    msg.z.extend(scan.xyz[:, 2].tolist())
    msg.intensity.extend(scan.intensity.tolist())


def scan_to_proto(scan: ScanData) -> "lidar_pb2.PointCloud3":
    msg = lidar_pb2.PointCloud3()
    _fill_scan(msg, scan)
    return msg


def imu_to_proto(s: ImuSample) -> "imu_pb2.IMUData":
    msg = imu_pb2.IMUData()
    msg.header.timestamp = s.t_ns
    msg.header.sequence_number = s.seq & 0xFFFFFFFF
    msg.ax, msg.ay, msg.az = (float(v) for v in s.accel_g)
    msg.gx, msg.gy, msg.gz = (float(v) for v in s.gyro)
    return msg


class SensorHub:
    """Both streams plus the client bookkeeping the engine needs."""

    def __init__(self, block: bool, with_lidar: bool = True, with_imu: bool = True) -> None:
        self.imu = Stream("IMU data", IMU_LIMITS, with_imu, block)
        self.lidar = Stream("Lidar scan", LIDAR_LIMITS, with_lidar, block)

    def on_imu(self, s: ImuSample) -> None:
        if self.imu.subscribers():
            self.imu.publish(s)

    def on_scan(self, scan: ScanData, gt: GroundTruth) -> None:
        if self.lidar.subscribers():
            self.lidar.publish(scan)

    def lidar_wanted(self) -> bool:
        return self.lidar.subscribers() > 0

    def wait_for_clients(self, abort: threading.Event, need_lidar: bool = True, need_imu: bool = True) -> None:
        """Block until every required stream has a subscriber (or `abort`)."""
        while not abort.is_set():
            if (not need_lidar or self.lidar.subscribers()) and (not need_imu or self.imu.subscribers()):
                return
            time.sleep(0.02)

    def close(self) -> None:
        self.imu.close_all()
        self.lidar.close_all()


def _serve_stream(stream: Stream, request_queue: int, context: grpc.ServicerContext, convert):
    if not stream.available:
        context.abort(grpc.StatusCode.UNAVAILABLE, f"{stream.name.split()[0]} not available")
    try:
        sub = stream.subscribe(request_queue)
    except OverflowError as e:
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, str(e))
    context.add_callback(sub.close)
    try:
        while context.is_active():
            item = sub.get(0.1)
            if item is CLOSED:
                return
            if item is not None:
                yield convert(item)
    finally:
        sub.close()


class _LidarServicer(lidar_pb2_grpc.LidarServiceServicer):
    def __init__(self, hub: SensorHub) -> None:
        self.hub = hub

    def getLidarScan(self, request, context):
        yield from _serve_stream(self.hub.lidar, request.max_queued_samples, context, scan_to_proto)


class _ImuServicer(imu_pb2_grpc.ImuServiceServicer):
    def __init__(self, hub: SensorHub) -> None:
        self.hub = hub

    def getImuData(self, request, context):
        yield from _serve_stream(self.hub.imu, request.max_queued_samples, context, imu_to_proto)


class SensorServer:
    def __init__(self, hub: SensorHub, listen: str = "0.0.0.0:50051") -> None:
        self.hub = hub
        # a stream occupies a worker for its whole life: 2 services x 8 clients, plus slack
        self._server = grpc.server(futures.ThreadPoolExecutor(max_workers=2 * MAX_CLIENTS_PER_STREAM + 4),
                                   options=[("grpc.max_send_message_length", 64 * 1024 * 1024)])
        lidar_pb2_grpc.add_LidarServiceServicer_to_server(_LidarServicer(hub), self._server)
        imu_pb2_grpc.add_ImuServiceServicer_to_server(_ImuServicer(hub), self._server)
        self.port = self._server.add_insecure_port(listen)
        if self.port == 0:
            raise RuntimeError(f"cannot listen on {listen}")

    def start(self) -> None:
        self._server.start()
        log.info("Sensor server listening on port %d", self.port)

    def stop(self, grace: float = 1.0) -> None:
        self.hub.close()
        self._server.stop(grace).wait()
