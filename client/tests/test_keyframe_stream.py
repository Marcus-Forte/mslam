import unittest
from typing import Any

import grpc

from keyframe_store import KeyframeChange, KeyframeResume, KeyframeStore
from keyframe_stream import stream_keyframes, uses_pose_graph
from map_stream import ServerTooOldError, StreamEnd
from tests.helpers import make_added, make_epoch, make_pose, make_poses, slam_pb2
from tests.test_map_stream import FakeCall, FakeRpcError


class FakeStub:
    def __init__(self, *calls: FakeCall, status: Any = None) -> None:
        self._calls = list(calls)
        self._status = status
        self.requests: list[slam_pb2.KeyframeStreamRequest] = []

    def StreamKeyframes(self, request: slam_pb2.KeyframeStreamRequest) -> FakeCall:  # noqa: N802 (gRPC stub name)
        self.requests.append(request)
        return self._calls.pop(0)

    def GetStatus(self, request: Any, timeout: float = 0.0) -> Any:  # noqa: N802 (gRPC stub name)
        if isinstance(self._status, Exception):
            raise self._status
        return self._status


class StreamKeyframesTest(unittest.TestCase):
    def test_reconnect_resumes_from_the_stored_position_and_keeps_the_keyframes(self) -> None:
        store = KeyframeStore()
        first = FakeCall([make_epoch(7), make_added(7, 0), make_added(7, 1), FakeRpcError(grpc.StatusCode.UNAVAILABLE)])
        second = FakeCall([make_epoch(7), make_added(7, 2)])
        stub = FakeStub(first, second)

        with self.assertRaises(FakeRpcError):
            stream_keyframes(stub, store)
        self.assertEqual(store.keyframe_count, 2)  # still there while disconnected
        self.assertEqual(stream_keyframes(stub, store), StreamEnd.COMPLETED)

        self.assertEqual((stub.requests[0].epoch, stub.requests[0].next_keyframe_id), (0, 0))
        self.assertEqual((stub.requests[1].epoch, stub.requests[1].next_keyframe_id), (7, 2))
        self.assertEqual(store.resume_position, KeyframeResume(7, 3))

    def test_protocol_violation_cancels_the_call_and_the_next_request_starts_over(self) -> None:
        store = KeyframeStore()
        call = FakeCall([make_epoch(7), make_added(7, 0), make_added(7, 0), make_added(7, 1)])
        stub = FakeStub(call, FakeCall([]))
        with self.assertLogs("keyframe_store", level="ERROR"):
            self.assertEqual(stream_keyframes(stub, store), StreamEnd.RESYNC)
        self.assertTrue(call.cancelled)
        stream_keyframes(stub, store)
        self.assertEqual((stub.requests[1].epoch, stub.requests[1].next_keyframe_id), (0, 0))

    def test_unimplemented_becomes_server_too_old(self) -> None:
        stub = FakeStub(FakeCall([FakeRpcError(grpc.StatusCode.UNIMPLEMENTED)]))
        with self.assertRaises(ServerTooOldError):
            stream_keyframes(stub, KeyframeStore())

    def test_on_change_and_should_stop(self) -> None:
        store = KeyframeStore()
        changes: list[KeyframeChange] = []
        call = FakeCall([make_epoch(7), make_added(7, 0), make_poses(7, 2, [make_pose(x=1.0)]), make_added(7, 1)])
        stops = iter([False, False, False, True])
        end = stream_keyframes(FakeStub(call), store, on_change=changes.append, should_stop=lambda: next(stops))
        self.assertEqual(end, StreamEnd.STOPPED)
        self.assertTrue(call.cancelled)
        self.assertEqual([(c.cleared, c.added, c.moved) for c in changes], [(True, 0, 0), (False, 1, 0), (False, 0, 1)])
        self.assertEqual(store.resume_position, KeyframeResume(7, 1))


class StrategyChoiceTest(unittest.TestCase):
    def test_pose_graph_strategy_selects_the_keyframe_stream(self) -> None:
        status = slam_pb2.SlamStatus(strategy=slam_pb2.SLAM_STRATEGY_POSE_GRAPH)
        self.assertTrue(uses_pose_graph(FakeStub(status=status)))

    def test_odometry_strategy_selects_the_map_stream(self) -> None:
        self.assertFalse(uses_pose_graph(FakeStub(status=slam_pb2.SlamStatus())))

    def test_a_server_without_get_status_is_an_odometry_server(self) -> None:
        self.assertFalse(uses_pose_graph(FakeStub(status=FakeRpcError(grpc.StatusCode.UNIMPLEMENTED))))

    def test_other_errors_propagate_so_the_caller_retries(self) -> None:
        with self.assertRaises(FakeRpcError):
            uses_pose_graph(FakeStub(status=FakeRpcError(grpc.StatusCode.UNAVAILABLE)))


if __name__ == "__main__":
    unittest.main()
