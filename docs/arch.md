# mslam Core Class Model

This document summarizes the main runtime structure around the core
classes: `Map` (via `IMap`), `Slam`, and `SlamServer` (gRPC endpoint).

At a high level:
- `Slam` owns the SLAM state machine (predict/update/reset).
- `Slam` depends on `IMap` for map storage and nearest-neighbor queries.
- `SlamServer` exposes map/pose/controls over gRPC and delegates control
	commands (`Start`, `Stop`, `Reset`) to a bound `Slam` instance.
- `VoxelHashMap` is the concrete `IMap` implementation and is selected by
	configuration.

```mermaid
classDiagram
direction LR

class ISlam {
	<<interface>>
	+ResetPose()
	+Predict(imuData)
	+Update(lidarData)
}

class IMap {
	<<interface>>
	+addScan(scan) PointCloud
	+getClosestNeighbor(query) Neighbor
	+getClosestNNeighbors(query, N) vector~Neighbor~
	+getPointCloudRepresentation() PointCloud
	+clear()
}

class Slam {
	+Slam(logger, config, map)
	+run(lidar, imu, server)
	+startProcessing()
	+stopProcessing()
	+reset()
	+isRunning() bool
	+getPose() Pose3D
	+getTransform() Affine3d
	-config_ SlamConfiguration
	-map_ shared_ptr~IMap~
	-dense_map_ unique_ptr~IMap~
	-registration_ unique_ptr~IRegistration~
	-imu_registration_ unique_ptr~ImuRegistration~
}

class IRegistration {
	<<interface>>
	+Align(state, map, scan) SlamState
}

class ImuRegistration {
	+Align(state, map, scan) SlamState
	+Align(state, map, scan, prev_state, preintegrator) SlamState
}

class VoxelHashMap

class SlamService_Service {
	<<gRPC generated base>>
	+GetMap(...)
	+GetMapIncrements(...)
	+GetTransformedScan(...)
	+GetCorrespondences(...)
	+GetPose(...)
	+Start(...)
	+Stop(...)
	+Reset(...)
}

class SlamServer {
	+start()
	+stop()
	+setSlam(slam)
	+updatePose(pose)
	+updateMapIncrement(increment)
	+updateTransformedScan(scan)
	+updateCorrespondences(correspondences)
	-map_history_ vector~shared_ptr~PointCloud~~
	-slam_ Slam*
	-server_ unique_ptr~grpc::Server~
}

ISlam <|.. Slam
IMap <|.. VoxelHashMap
IRegistration <|.. ImuRegistration
SlamService_Service <|-- SlamServer

Slam --> IMap : uses for map ops
Slam --> IRegistration : scan matching
Slam --> ImuRegistration : IMU+LiDAR optimization
Slam ..> SlamServer : publishes pose/scan/map increment
SlamServer --> Slam : control calls\nStart/Stop/Reset
SlamServer ..> IMap : registration map is bounded\n(full map accumulated separately)
```

## Notes

- `SlamServer` is both a transport adapter (gRPC) and a live data publisher
	for pose and point-cloud streams.
- `Slam` remains the computation core and is transport-agnostic, except for
	pushing outputs to `SlamServer` during `run()`.
- The map abstraction allows switching map backends without changing the SLAM
	pipeline logic.
