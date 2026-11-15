import lidar_pb2 as _lidar_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

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
