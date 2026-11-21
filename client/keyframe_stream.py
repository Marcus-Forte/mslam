"""The gRPC side of the keyframe stream: one `StreamKeyframes` call applied to a `KeyframeStore`, and the choice
between it and `StreamMap` from `GetStatus`.

Kept apart from the store (no gRPC there) and from the renderer (no viser here). The caller owns the channel, the
reconnect policy and the threads.
"""

import logging
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import grpc

from keyframe_store import KeyframeChange, KeyframeStore
from map_stream import ServerTooOldError, StreamEnd

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import slam_pb2  # noqa: E402

logger = logging.getLogger(__name__)


def uses_pose_graph(stub: Any, timeout: float = 2.0) -> bool:
    """True if the server runs the pose-graph strategy (`GetStatus.strategy`). A server too old to implement
    `GetStatus` is an odometry server: the viewer then streams the map as before. Other errors propagate."""
    try:
        status = stub.GetStatus(slam_pb2.Empty(), timeout=timeout)
    except grpc.RpcError as exc:
        if exc.code() == grpc.StatusCode.UNIMPLEMENTED:
            return False
        raise
    return status.strategy == slam_pb2.SLAM_STRATEGY_POSE_GRAPH


def stream_keyframes(
    stub: Any,
    store: KeyframeStore,
    on_change: Callable[[KeyframeChange], None] | None = None,
    should_stop: Callable[[], bool] = lambda: False,
) -> StreamEnd:
    """Open one `StreamKeyframes` from the store's resume position and apply messages until it ends.

    `on_change` is called after each applied message. A gRPC error propagates (the caller reconnects and the next
    request carries the store's `(epoch, next_keyframe_id)`), except UNIMPLEMENTED, which becomes
    `ServerTooOldError`.
    """
    epoch, next_id = store.resume_position
    request = slam_pb2.KeyframeStreamRequest(epoch=epoch, next_keyframe_id=next_id)
    logger.info("Streaming keyframes from epoch %d, keyframe %d", epoch, next_id)

    call = stub.StreamKeyframes(request)
    try:
        for message in call:
            if should_stop():
                call.cancel()
                return StreamEnd.STOPPED
            change = store.apply(message)
            logger.debug(
                "Keyframe message %s: +%d keyframes, %d moved",
                message.WhichOneof("payload"),
                change.added,
                change.moved,
            )
            if on_change is not None:
                on_change(change)
            if change.resync:
                call.cancel()
                return StreamEnd.RESYNC
    except grpc.RpcError as exc:
        if exc.code() == grpc.StatusCode.UNIMPLEMENTED:
            raise ServerTooOldError("the server does not implement StreamKeyframes") from exc
        raise
    return StreamEnd.COMPLETED
