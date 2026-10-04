# mslam Requirements

1. **MSLAM-REQ-001 — Operating modes:** The system shall support three modes: (a) local, with the sensor read in-process through an in-memory interface (`remote_scanner: "local"`); (b) remote, with the sensor read through a service client (`remote_scanner: "<host>:<port>"`); (c) replay of a recorded `.pbscan` file (`-f`, which takes precedence over the configured source).
2. **MSLAM-REQ-002 — Sensor processing:** The system shall estimate 6-DoF pose from LiDAR scans, optionally aided by IMU data, and support configured scan filtering, downsampling, and deskewing.
3. **MSLAM-REQ-003 — Map and pose output:** The system shall maintain a voxel map and publish pose, transformed-scan, and map-increment updates through its gRPC server.
4. **MSLAM-REQ-004 — Remote interface:** The gRPC interface shall provide the full-map query, pose/scan/map-increment streams, and Start, Stop, and Reset controls. Control calls shall not race the SLAM processing loop.
5. **MSLAM-REQ-005 — Per-scan deadline:** For every scan in local and remote modes, the full processing time, from dequeue through IMU prediction, preprocessing, pose optimization, map update, and publishing to the server, shall not exceed one sensor sampling interval (48 ms for the Mid360 at the configured `accumulate_scan_count`). No scan shall be dropped because the previous scan overran. Per-scan total time shall be logged so this can be verified.
6. **MSLAM-REQ-006 — Non-interference:** Logging and the gRPC server (including any number of attached clients and the full-map query) shall not cause any scan to miss the MSLAM-REQ-005 deadline. Logging at the shipped default level shall not perform blocking I/O on the processing path.
7. **MSLAM-REQ-007 — Pose accuracy:** On reference recordings with a ground-truth trajectory, at every scan, position error shall be no greater than 0.05 m and orientation error no greater than 5 degrees.
8. **MSLAM-REQ-008 — Configuration:** The system shall load sensor selection, source, logging level, preprocessing, map, and optimizer settings from the JSON configuration (`-c`), and Mid360 settings, including scan accumulation count, from the adjacent `publisher_config.json`. Invalid configuration shall be rejected at startup.
9. **MSLAM-REQ-009 — Map accuracy:** On reference recordings with a ground-truth map or surfaces, the published map (full-map query and accumulated increments) shall have an error no greater than 0.05 m for every point, including after Reset and re-mapping.

## Known gaps (current code)

- **MSLAM-REQ-005/006:** The total per-scan time is not measured or checked. The logger is synchronous (stdout), the shipped config uses `debug`, and the SLAM thread copies data to the server under mutexes.
- **MSLAM-REQ-004:** `GetCorrespondences` is declared but never published, and Start/Stop/Reset are unsynchronized with the SLAM loop.
- **MSLAM-REQ-007:** No ground-truth data exists. The only check is a final-position loop-closure offset of 0.1 m, with no rotation check (heading ends about 0.49 rad off on `living_room_garden`).
- **MSLAM-REQ-009:** No ground-truth map exists and map error is not measured. Map accuracy is also bounded by voxel resolution (`map.resolution` is 0.05 m in the shipped config but 0.5 m in the integration test config) and by map points inherited from pose error.
