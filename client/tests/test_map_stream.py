import unittest
from collections.abc import Iterable
from typing import Any

import grpc

from tests.helpers import make_chunk, slam_pb2
from map_store import MapChange, MapStore, ResumePosition
from map_stream import ServerTooOldError, StreamEnd, stream_map


class FakeCall:
    def __init__(self, items: Iterable[Any]) -> None:
        self._items = iter(items)
        self.cancelled = False

    def __iter__(self) -> "FakeCall":
        return self

    def __next__(self) -> Any:
        item = next(self._items)
        if isinstance(item, Exception):
            raise item
        return item

    def cancel(self) -> bool:
        self.cancelled = True
        return True


class FakeStub:
    def __init__(self, *calls: FakeCall) -> None:
        self._calls = list(calls)
        self.requests: list[slam_pb2.MapStreamRequest] = []

    def StreamMap(self, request: slam_pb2.MapStreamRequest) -> FakeCall:  # noqa: N802 (gRPC stub name)
        self.requests.append(request)
        return self._calls.pop(0)


class FakeRpcError(grpc.RpcError):
    def __init__(self, code: grpc.StatusCode) -> None:
        super().__init__()
        self._code = code

    def code(self) -> grpc.StatusCode:
        return self._code


class StreamMapTest(unittest.TestCase):
    def test_reconnect_resumes_from_the_stored_position_and_keeps_the_map(self) -> None:
        store = MapStore(capacity=100)
        first = FakeCall([make_chunk(7, 0, 0), make_chunk(7, 0, 3, count=6), FakeRpcError(grpc.StatusCode.UNAVAILABLE)])
        second = FakeCall([make_chunk(7, 3, 3), make_chunk(7, 3, 5, count=4, start=6)])
        stub = FakeStub(first, second)

        with self.assertRaises(FakeRpcError):
            stream_map(stub, store)
        self.assertEqual(store.point_count, 6)  # still there while disconnected
        self.assertEqual(stream_map(stub, store), StreamEnd.COMPLETED)

        self.assertEqual((stub.requests[0].epoch, stub.requests[0].start_offset), (0, 0))
        self.assertEqual((stub.requests[1].epoch, stub.requests[1].start_offset), (7, 3))
        self.assertEqual(store.point_count, 10)
        self.assertEqual(store.resume_position, ResumePosition(7, 5))

    def test_protocol_violation_cancels_the_call_and_the_next_request_starts_over(self) -> None:
        store = MapStore(capacity=100)
        call = FakeCall([make_chunk(7, 0, 4, count=4), make_chunk(7, 2, 5, count=3), make_chunk(7, 5, 6, count=1)])
        stub = FakeStub(call, FakeCall([]))
        with self.assertLogs("map_store", level="ERROR"):
            self.assertEqual(stream_map(stub, store), StreamEnd.RESYNC)
        self.assertTrue(call.cancelled)
        stream_map(stub, store)
        self.assertEqual((stub.requests[1].epoch, stub.requests[1].start_offset), (0, 0))

    def test_unimplemented_becomes_server_too_old(self) -> None:
        stub = FakeStub(FakeCall([FakeRpcError(grpc.StatusCode.UNIMPLEMENTED)]))
        with self.assertRaises(ServerTooOldError):
            stream_map(stub, MapStore(capacity=10))

    def test_on_change_and_should_stop(self) -> None:
        store = MapStore(capacity=100)
        changes: list[MapChange] = []
        call = FakeCall([make_chunk(7, 0, 0), make_chunk(7, 0, 1, count=2), make_chunk(7, 1, 2, count=2, start=2)])
        stops = iter([False, False, True])
        end = stream_map(FakeStub(call), store, on_change=changes.append, should_stop=lambda: next(stops))
        self.assertEqual(end, StreamEnd.STOPPED)
        self.assertTrue(call.cancelled)
        self.assertEqual([c.appended_points for c in changes], [0, 2])
        self.assertEqual(store.resume_position, ResumePosition(7, 1))


if __name__ == "__main__":
    unittest.main()
