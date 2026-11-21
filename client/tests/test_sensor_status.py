"""Sensor LED: state mapping, HTML, and the status monitor against a real in-process gRPC server."""

import threading
import time
import unittest
from concurrent import futures

import grpc

from tests import helpers  # noqa: F401  (puts the client directory on sys.path)

from proto_gen import slam_pb2, slam_pb2_grpc  # noqa: E402
from sensor_status import (  # noqa: E402
    GraphInfo,
    SensorState,
    SensorStatusMonitor,
    graph_info_from_status,
    graph_led_html,
    sensor_led_html,
    state_from_status,
)

S = SensorState


class FakeSlam(slam_pb2_grpc.SlamServiceServicer):
    """Answers GetStatus with whatever `status` holds."""

    def __init__(self) -> None:
        self.status = slam_pb2.SlamStatus()

    def GetStatus(self, request, context):  # noqa: N802  (gRPC method name)
        return self.status


class OldSlam(slam_pb2_grpc.SlamServiceServicer):
    """A server from before GetStatus: the inherited method answers UNIMPLEMENTED."""


def start_server(servicer: slam_pb2_grpc.SlamServiceServicer, port: int = 0) -> tuple[grpc.Server, int]:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    slam_pb2_grpc.add_SlamServiceServicer_to_server(servicer, server)
    port = server.add_insecure_port(f"127.0.0.1:{port}")
    server.start()
    return server, port


class StateMappingTest(unittest.TestCase):
    def test_every_link_maps_to_its_state(self) -> None:
        expected = {
            slam_pb2.SENSOR_LINK_STREAMING: S.STREAMING,
            slam_pb2.SENSOR_LINK_CONNECTED_IDLE: S.IDLE,
            slam_pb2.SENSOR_LINK_DISCONNECTED: S.DISCONNECTED,
            slam_pb2.SENSOR_LINK_NOT_APPLICABLE: S.NOT_APPLICABLE,
            slam_pb2.SENSOR_LINK_UNKNOWN: S.UNKNOWN,
        }
        for link, state in expected.items():
            self.assertIs(state_from_status(slam_pb2.SlamStatus(sensor_link=link)), state, link)

    def test_a_link_from_a_newer_server_is_unknown(self) -> None:
        self.assertIs(state_from_status(slam_pb2.SlamStatus(sensor_link=99)), S.UNKNOWN)

    def test_colours_distinguish_the_sensor_states(self) -> None:
        colours = {S.STREAMING.color, S.IDLE.color, S.DISCONNECTED.color, S.UNKNOWN.color}
        self.assertEqual(len(colours), 4)

    def test_html_shows_state_and_escapes_the_address(self) -> None:
        text = sensor_led_html(S.STREAMING, "<host>:1")
        self.assertIn(S.STREAMING.color, text)
        self.assertIn("streaming", text)
        self.assertIn("&lt;host&gt;:1", text)
        self.assertNotIn("<host>", text)

    def test_html_without_an_address(self) -> None:
        text = sensor_led_html(S.SERVER_UNREACHABLE, "")
        self.assertIn("SLAM server unreachable", text)
        self.assertNotIn("<code>", text)


class GraphInfoTest(unittest.TestCase):
    def test_odometry_status_has_no_graph(self) -> None:
        info = graph_info_from_status(slam_pb2.SlamStatus(keyframes=5))
        self.assertEqual(info, GraphInfo())
        self.assertEqual(graph_led_html(info), "")

    def test_pose_graph_status_shows_keyframes_and_loop_closures(self) -> None:
        status = slam_pb2.SlamStatus(strategy=slam_pb2.SLAM_STRATEGY_POSE_GRAPH, keyframes=74, loop_closures=32)
        info = graph_info_from_status(status)
        self.assertEqual(info, GraphInfo(True, 74, 32))
        text = graph_led_html(info)
        self.assertIn("74 keyframes", text)
        self.assertIn("32 loop closures", text)
        self.assertIn("1 loop closure<", graph_led_html(GraphInfo(True, 3, 1)) + "<")


class MonitorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.stop = threading.Event()
        self.changes: list[tuple[SensorState, str]] = []
        self.changed = threading.Condition()
        self.servers: list[grpc.Server] = []
        self.channels: list[grpc.Channel] = []

    def tearDown(self) -> None:
        self.stop.set()
        for channel in self.channels:
            channel.close()
        for server in self.servers:
            server.stop(grace=None)

    def on_change(self, state: SensorState, address: str) -> None:
        with self.changed:
            self.changes.append((state, address))
            self.changed.notify_all()

    def monitor_for(self, port: int) -> SensorStatusMonitor:
        channel = grpc.insecure_channel(f"127.0.0.1:{port}")
        self.channels.append(channel)
        monitor = SensorStatusMonitor(
            slam_pb2_grpc.SlamServiceStub(channel), self.on_change, poll_interval=0.05, rpc_timeout=1.0
        )
        monitor.start(self.stop.is_set, self.stop.wait)
        return monitor

    def wait_for(self, state: SensorState, timeout: float = 6.0) -> None:
        with self.changed:
            ok = self.changed.wait_for(lambda: self.changes and self.changes[-1][0] is state, timeout)
        self.assertTrue(ok, f"no {state} within {timeout} s; changes {self.changes}")

    def test_reports_graph_changes_to_on_graph(self) -> None:
        slam = FakeSlam()
        server, port = start_server(slam)
        self.servers.append(server)
        graphs: list[GraphInfo] = []
        done = threading.Event()

        def on_graph(info: GraphInfo) -> None:
            graphs.append(info)
            if info.keyframes == 9:
                done.set()

        channel = grpc.insecure_channel(f"127.0.0.1:{port}")
        self.channels.append(channel)
        monitor = SensorStatusMonitor(
            slam_pb2_grpc.SlamServiceStub(channel),
            self.on_change,
            poll_interval=0.05,
            rpc_timeout=1.0,
            on_graph=on_graph,
        )
        monitor.start(self.stop.is_set, self.stop.wait)
        self.assertEqual(graphs[0], GraphInfo())  # once at start

        slam.status = slam_pb2.SlamStatus(strategy=slam_pb2.SLAM_STRATEGY_POSE_GRAPH, keyframes=9, loop_closures=2)
        self.assertTrue(done.wait(6.0), graphs)
        self.assertEqual(graphs[-1], GraphInfo(True, 9, 2))
        self.assertEqual(len(graphs), 2)  # no repeat while it stays the same

    def test_follows_the_servers_sensor_link(self) -> None:
        slam = FakeSlam()
        server, port = start_server(slam)
        self.servers.append(server)
        self.monitor_for(port)
        self.assertEqual(self.changes[0], (S.UNKNOWN, ""))  # reported once at start

        slam.status = slam_pb2.SlamStatus(
            sensor_link=slam_pb2.SENSOR_LINK_CONNECTED_IDLE, sensor_address="10.0.0.2:50051"
        )
        self.wait_for(S.IDLE)
        self.assertEqual(self.changes[-1], (S.IDLE, "10.0.0.2:50051"))

        slam.status = slam_pb2.SlamStatus(
            sensor_link=slam_pb2.SENSOR_LINK_STREAMING, sensor_address="10.0.0.2:50051", scan_age_ms=40, imu_age_ms=3
        )
        self.wait_for(S.STREAMING)

        slam.status = slam_pb2.SlamStatus(
            sensor_link=slam_pb2.SENSOR_LINK_DISCONNECTED, sensor_address="10.0.0.2:50051"
        )
        self.wait_for(S.DISCONNECTED)

    def test_only_changes_are_reported(self) -> None:
        slam = FakeSlam()
        slam.status = slam_pb2.SlamStatus(sensor_link=slam_pb2.SENSOR_LINK_STREAMING, sensor_address="a:1")
        server, port = start_server(slam)
        self.servers.append(server)
        self.monitor_for(port)
        self.wait_for(S.STREAMING)
        time.sleep(0.4)  # several more polls of the same answer
        self.assertEqual([state for state, _ in self.changes], [S.UNKNOWN, S.STREAMING])

    def test_slam_server_going_away_is_unknown_not_disconnected(self) -> None:
        slam = FakeSlam()
        slam.status = slam_pb2.SlamStatus(sensor_link=slam_pb2.SENSOR_LINK_STREAMING, sensor_address="a:1")
        server, port = start_server(slam)
        self.monitor_for(port)
        self.wait_for(S.STREAMING)
        server.stop(grace=None).wait()
        self.wait_for(S.SERVER_UNREACHABLE)
        # And it recovers when the SLAM server is back on the same port.
        server, _ = start_server(slam, port)
        self.servers.append(server)
        self.wait_for(S.STREAMING)

    def test_a_server_without_get_status_is_too_old(self) -> None:
        server, port = start_server(OldSlam())
        self.servers.append(server)
        self.monitor_for(port)
        self.wait_for(S.SERVER_TOO_OLD)

    def test_on_board_sensor(self) -> None:
        slam = FakeSlam()
        slam.status = slam_pb2.SlamStatus(sensor_link=slam_pb2.SENSOR_LINK_NOT_APPLICABLE, sensor_address="local")
        server, port = start_server(slam)
        self.servers.append(server)
        self.monitor_for(port)
        self.wait_for(S.NOT_APPLICABLE)
        self.assertEqual(self.changes[-1][1], "local")


if __name__ == "__main__":
    unittest.main()
