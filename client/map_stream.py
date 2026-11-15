"""The gRPC side of the map stream: one `StreamMap` call applied to a `MapStore`.

Kept apart from the store (no gRPC there) and from the renderer (no viser here). The caller owns the channel, the
reconnect policy and the threads.
"""

import enum
import logging
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import grpc

from map_store import MapChange, MapStore

PROTO_GEN_DIR = Path(__file__).resolve().parent / "proto_gen"
if str(PROTO_GEN_DIR) not in sys.path:
    sys.path.insert(0, str(PROTO_GEN_DIR))

from proto_gen import slam_pb2  # noqa: E402

logger = logging.getLogger(__name__)


class ServerTooOldError(RuntimeError):
    """The server does not implement `StreamMap` (gRPC status UNIMPLEMENTED)."""


class StreamEnd(enum.Enum):
    COMPLETED = "completed"  # the server ended the stream
    STOPPED = "stopped"  # `should_stop` returned True
    RESYNC = "resync"  # protocol violation; reconnect at once, the store now asks for offset 0


def stream_map(
    stub: Any,
    store: MapStore,
    on_change: Callable[[MapChange], None] | None = None,
    should_stop: Callable[[], bool] = lambda: False,
    max_points_per_chunk: int = 0,
) -> StreamEnd:
    """Open one `StreamMap` from the store's resume position and apply chunks until it ends.

    `on_change` is called after each applied chunk. A gRPC error propagates (the caller reconnects and the next
    request carries the store's `(epoch, next_offset)`), except UNIMPLEMENTED, which becomes `ServerTooOldError`.
    """
    epoch, next_offset = store.resume_position
    request = slam_pb2.MapStreamRequest(
        epoch=epoch, start_offset=next_offset, max_points_per_chunk=max_points_per_chunk
    )
    logger.info("Streaming map from epoch %d, offset %d", epoch, next_offset)

    call = stub.StreamMap(request)
    try:
        for chunk in call:
            if should_stop():
                call.cancel()
                return StreamEnd.STOPPED
            change = store.apply(chunk)
            logger.debug(
                "Map chunk epoch %d [%d, %d): %d points",
                chunk.epoch,
                chunk.first_offset,
                chunk.next_offset,
                change.appended_points,
            )
            if on_change is not None:
                on_change(change)
            if change.resync:
                call.cancel()
                return StreamEnd.RESYNC
    except grpc.RpcError as exc:
        if exc.code() == grpc.StatusCode.UNIMPLEMENTED:
            raise ServerTooOldError("the server does not implement StreamMap") from exc
        raise
    return StreamEnd.COMPLETED
