import lidar_pb2 as _lidar_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SensorLink(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    SENSOR_LINK_UNKNOWN: _ClassVar[SensorLink]
    SENSOR_LINK_NOT_APPLICABLE: _ClassVar[SensorLink]
    SENSOR_LINK_DISCONNECTED: _ClassVar[SensorLink]
    SENSOR_LINK_CONNECTED_IDLE: _ClassVar[SensorLink]
    SENSOR_LINK_STREAMING: _ClassVar[SensorLink]

class SlamStrategy(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    SLAM_STRATEGY_ODOMETRY: _ClassVar[SlamStrategy]
    SLAM_STRATEGY_POSE_GRAPH: _ClassVar[SlamStrategy]
SENSOR_LINK_UNKNOWN: SensorLink
SENSOR_LINK_NOT_APPLICABLE: SensorLink
SENSOR_LINK_DISCONNECTED: SensorLink
SENSOR_LINK_CONNECTED_IDLE: SensorLink
SENSOR_LINK_STREAMING: SensorLink
SLAM_STRATEGY_ODOMETRY: SlamStrategy
SLAM_STRATEGY_POSE_GRAPH: SlamStrategy

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Pose3D(_message.Message):
    __slots__ = ("x", "y", "z", "phi", "omega", "theta")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    PHI_FIELD_NUMBER: _ClassVar[int]
    OMEGA_FIELD_NUMBER: _ClassVar[int]
    THETA_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    phi: float
    omega: float
    theta: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ..., phi: _Optional[float] = ..., omega: _Optional[float] = ..., theta: _Optional[float] = ...) -> None: ...

class MapStreamRequest(_message.Message):
    __slots__ = ("epoch", "start_offset", "max_points_per_chunk")
    EPOCH_FIELD_NUMBER: _ClassVar[int]
    START_OFFSET_FIELD_NUMBER: _ClassVar[int]
    MAX_POINTS_PER_CHUNK_FIELD_NUMBER: _ClassVar[int]
    epoch: int
    start_offset: int
    max_points_per_chunk: int
    def __init__(self, epoch: _Optional[int] = ..., start_offset: _Optional[int] = ..., max_points_per_chunk: _Optional[int] = ...) -> None: ...

class MapStreamChunk(_message.Message):
    __slots__ = ("epoch", "first_offset", "next_offset", "points")
    EPOCH_FIELD_NUMBER: _ClassVar[int]
    FIRST_OFFSET_FIELD_NUMBER: _ClassVar[int]
    NEXT_OFFSET_FIELD_NUMBER: _ClassVar[int]
    POINTS_FIELD_NUMBER: _ClassVar[int]
    epoch: int
    first_offset: int
    next_offset: int
    points: _lidar_pb2.PointCloud3
    def __init__(self, epoch: _Optional[int] = ..., first_offset: _Optional[int] = ..., next_offset: _Optional[int] = ..., points: _Optional[_Union[_lidar_pb2.PointCloud3, _Mapping]] = ...) -> None: ...

class PoseQuat(_message.Message):
    __slots__ = ("x", "y", "z", "qw", "qx", "qy", "qz")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    QW_FIELD_NUMBER: _ClassVar[int]
    QX_FIELD_NUMBER: _ClassVar[int]
    QY_FIELD_NUMBER: _ClassVar[int]
    QZ_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ..., qw: _Optional[float] = ..., qx: _Optional[float] = ..., qy: _Optional[float] = ..., qz: _Optional[float] = ...) -> None: ...

class KeyframeStreamRequest(_message.Message):
    __slots__ = ("epoch", "next_keyframe_id")
    EPOCH_FIELD_NUMBER: _ClassVar[int]
    NEXT_KEYFRAME_ID_FIELD_NUMBER: _ClassVar[int]
    epoch: int
    next_keyframe_id: int
    def __init__(self, epoch: _Optional[int] = ..., next_keyframe_id: _Optional[int] = ...) -> None: ...

class KeyframeEpoch(_message.Message):
    __slots__ = ("epoch",)
    EPOCH_FIELD_NUMBER: _ClassVar[int]
    epoch: int
    def __init__(self, epoch: _Optional[int] = ...) -> None: ...

class KeyframeAdded(_message.Message):
    __slots__ = ("epoch", "id", "pose", "cloud")
    EPOCH_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    CLOUD_FIELD_NUMBER: _ClassVar[int]
    epoch: int
    id: int
    pose: PoseQuat
    cloud: _lidar_pb2.PointCloud3
    def __init__(self, epoch: _Optional[int] = ..., id: _Optional[int] = ..., pose: _Optional[_Union[PoseQuat, _Mapping]] = ..., cloud: _Optional[_Union[_lidar_pb2.PointCloud3, _Mapping]] = ...) -> None: ...

class KeyframePoses(_message.Message):
    __slots__ = ("epoch", "graph_version", "poses")
    EPOCH_FIELD_NUMBER: _ClassVar[int]
    GRAPH_VERSION_FIELD_NUMBER: _ClassVar[int]
    POSES_FIELD_NUMBER: _ClassVar[int]
    epoch: int
    graph_version: int
    poses: _containers.RepeatedCompositeFieldContainer[KeyframePoseEntry]
    def __init__(self, epoch: _Optional[int] = ..., graph_version: _Optional[int] = ..., poses: _Optional[_Iterable[_Union[KeyframePoseEntry, _Mapping]]] = ...) -> None: ...

class KeyframePoseEntry(_message.Message):
    __slots__ = ("id", "pose")
    ID_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    id: int
    pose: PoseQuat
    def __init__(self, id: _Optional[int] = ..., pose: _Optional[_Union[PoseQuat, _Mapping]] = ...) -> None: ...

class KeyframeStreamMessage(_message.Message):
    __slots__ = ("epoch_reset", "added", "poses")
    EPOCH_RESET_FIELD_NUMBER: _ClassVar[int]
    ADDED_FIELD_NUMBER: _ClassVar[int]
    POSES_FIELD_NUMBER: _ClassVar[int]
    epoch_reset: KeyframeEpoch
    added: KeyframeAdded
    poses: KeyframePoses
    def __init__(self, epoch_reset: _Optional[_Union[KeyframeEpoch, _Mapping]] = ..., added: _Optional[_Union[KeyframeAdded, _Mapping]] = ..., poses: _Optional[_Union[KeyframePoses, _Mapping]] = ...) -> None: ...

class SlamStatus(_message.Message):
    __slots__ = ("sensor_link", "sensor_address", "scan_age_ms", "imu_age_ms", "strategy", "graph_version", "keyframes", "loop_closures")
    SENSOR_LINK_FIELD_NUMBER: _ClassVar[int]
    SENSOR_ADDRESS_FIELD_NUMBER: _ClassVar[int]
    SCAN_AGE_MS_FIELD_NUMBER: _ClassVar[int]
    IMU_AGE_MS_FIELD_NUMBER: _ClassVar[int]
    STRATEGY_FIELD_NUMBER: _ClassVar[int]
    GRAPH_VERSION_FIELD_NUMBER: _ClassVar[int]
    KEYFRAMES_FIELD_NUMBER: _ClassVar[int]
    LOOP_CLOSURES_FIELD_NUMBER: _ClassVar[int]
    sensor_link: SensorLink
    sensor_address: str
    scan_age_ms: int
    imu_age_ms: int
    strategy: SlamStrategy
    graph_version: int
    keyframes: int
    loop_closures: int
    def __init__(self, sensor_link: _Optional[_Union[SensorLink, str]] = ..., sensor_address: _Optional[str] = ..., scan_age_ms: _Optional[int] = ..., imu_age_ms: _Optional[int] = ..., strategy: _Optional[_Union[SlamStrategy, str]] = ..., graph_version: _Optional[int] = ..., keyframes: _Optional[int] = ..., loop_closures: _Optional[int] = ...) -> None: ...
