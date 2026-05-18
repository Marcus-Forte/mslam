---
name: regression
description: "Run a SLAM regression test: build, replay living_room.pbscan, extract the final logged pose, and compare against the known reference. Use when verifying SLAM correctness after code changes."
---

# SLAM Regression Test

## When to Use

- After modifying registration, preprocessing, map, or SLAM pipeline code
- Before merging changes that touch numerical paths
- To verify the system still converges to the expected trajectory

## Procedure

1. **Build the target**

   ```bash
   cmake --build build/default --target mslam
   ```

2. **Run the SLAM playback**

   ```bash
   cd /workspaces/mslam && ./build/default/mslam -c config/mslam.json -f living_room.pbscan -d 0 2>&1 | grep "State:" | tail -1
   ```

3. **Extract the final state** from the last `[Info][Slam.cc:38]: State:` log line.

4. **Compare against the reference pose:**

   ```
   pos=[0.150,-0.205,0.012] rot=[0.001,0.007,1.379] vel=[0.044,-0.043,-0.130] bg=[0.0001,-0.0000,0.0001] ba=[0.0000,-0.0000,0.0000]
   ```

5. **Pass/Fail criteria:**
   - Position (`pos`) components must match within **±0.05 m**
   - Rotation (`rot`) components must match within **±0.03 rad**
   - Velocity (`vel`) components must match within **±0.10 m/s**
   - Bias terms (`bg`, `ba`) must match within **±0.005**

   If any component exceeds its tolerance, the regression **fails**—report which component drifted and by how much.

## Interpretation

- A position drift suggests registration or map insertion changed behavior.
- A rotation drift often points to normal estimation or correspondence weighting changes.
- Bias drift indicates IMU preintegration or state propagation changes.

## Notes

- The `-d 0` flag sets playback delay to lowest, required to run the test as fast as possible.
- The recording `living_room.pbscan` is the canonical regression dataset.
- Config is `config/mslam.json`; if config fields changed, verify alignment first.