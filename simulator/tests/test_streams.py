import threading

import grpc
import pytest

from msim.streams import (CLOSED, IMU_LIMITS, LIDAR_LIMITS, MAX_CLIENTS_PER_STREAM, SensorHub, SensorServer,
                          Stream, resolve_queue_size)
import imu_pb2
import imu_pb2_grpc
import lidar_pb2
import lidar_pb2_grpc


def test_resolve_queue_size_matches_the_real_server():
    assert resolve_queue_size(0, IMU_LIMITS) == 200
    assert resolve_queue_size(1000, IMU_LIMITS) == 1000
    assert resolve_queue_size(10**6, IMU_LIMITS) == 4000
    assert resolve_queue_size(0, LIDAR_LIMITS) == 1
    assert resolve_queue_size(5, LIDAR_LIMITS) == 5
    assert resolve_queue_size(1000, LIDAR_LIMITS) == 64


def test_drop_oldest_when_full():
    s = Stream("t", IMU_LIMITS, True, block=False)
    sub = s.subscribe(3)
    ev = threading.Event()
    for i in range(10):
        sub.put(i, ev)
    assert [sub.get(0.1) for _ in range(3)] == [7, 8, 9]
    assert sub.dropped == 7
    assert sub.get(0.01) is None


def test_block_mode_waits_for_room():
    s = Stream("t", IMU_LIMITS, True, block=True)
    sub = s.subscribe(2)
    ev = threading.Event()
    sub.put(0, ev)
    sub.put(1, ev)
    done = threading.Event()
    t = threading.Thread(target=lambda: (sub.put(2, ev), done.set()))
    t.start()
    assert not done.wait(0.3)                       # producer is held back
    assert sub.get(0.1) == 0
    assert done.wait(2.0)
    t.join()
    assert [sub.get(0.1), sub.get(0.1)] == [1, 2] and sub.dropped == 0


def test_max_clients_and_close():
    s = Stream("t", IMU_LIMITS, True, block=False)
    subs = [s.subscribe(1) for _ in range(MAX_CLIENTS_PER_STREAM)]
    with pytest.raises(OverflowError):
        s.subscribe(1)
    subs[0].close()
    s.subscribe(1)                                  # a slot is free again
    s.close_all()
    assert subs[1].get(0.1) is CLOSED


def test_grpc_status_codes():
    hub = SensorHub(block=False, with_lidar=False)
    server = SensorServer(hub, "127.0.0.1:0")
    server.start()
    try:
        ch = grpc.insecure_channel(f"127.0.0.1:{server.port}")
        with pytest.raises(grpc.RpcError) as e:
            next(iter(lidar_pb2_grpc.LidarServiceStub(ch).getLidarScan(lidar_pb2.LidarStreamRequest())))
        assert e.value.code() == grpc.StatusCode.UNAVAILABLE
        calls = [imu_pb2_grpc.ImuServiceStub(ch).getImuData(imu_pb2.ImuStreamRequest(max_queued_samples=10))
                 for _ in range(MAX_CLIENTS_PER_STREAM + 1)]
        # the 9 calls race; whichever one loses must end with RESOURCE_EXHAUSTED, the other 8 stay open
        import time
        deadline = time.time() + 10
        rejected = []
        while time.time() < deadline and not rejected:
            rejected = [c for c in calls if c.done()]
            time.sleep(0.02)
        assert len(rejected) == 1
        assert rejected[0].code() == grpc.StatusCode.RESOURCE_EXHAUSTED
        assert hub.imu.subscribers() == MAX_CLIENTS_PER_STREAM
        for c in calls:
            c.cancel()
        ch.close()
    finally:
        server.stop(0.1)
