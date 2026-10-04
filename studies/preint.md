The IMU dead-reckoning in Slam::Predict (src/slam/Slam.cc) was integrating attitude incorrectly, and it disagreed with the preintegrator in the fused optimizer:
// old (broken) code
state_.rotation.x() += dt * imuData.gx;   // Euler += body rate
state_.rotation.y() += dt * imuData.gy;
state_.rotation.z() += dt * imuData.gz;
Two problems:
1. Euler-angle accumulation of body rates instead of SO(3) composition. Adding three body rates to three Euler angles is only valid for tiny rotations about a single axis near zero attitude. Under fast rotation it is wrong by O(θ³) and, worse, the order of axes matters.
2. No bias subtraction in the dead-reckoning path. Slam::Predict rotated with the raw gyro (gx,gy,gz) and gravity-compensated with the raw accelerometer, while ImuPreintegrator correctly uses gyro - bg and accel - ba. So the prediction used by scan-matching drifted from the measurement model the optimizer was fitting.
That dead-reckoned pose is not just cosmetic: it is the initial guess and the cloud transform fed to registration, and it drives correspondence search (gate = 0.15 m). Once fast motion pushed the Euler-integrated prior past the correspondence gate, registration latched onto wrong matches, the resulting bad pose was inserted into the map, and the trajectory followed the corrupted geometry — the vertical runaway.
Evidence (living_room.pbscan, 1345 scan updates)
Config	Before	After fix
deskew: off, res 0.1	final pos (-0.31, -0.37, **-12.26**), vel z spiked to -9.5 m/s	(0.14, 0.31, **0.00**)
working tree (constant_velocity, res 0.05)	(4.01, 0.63, **-8.42**)	(-0.05, 0.32, **-0.04**)
high-frequency jitter (p99 2nd-diff)	0.235 / 0.378 m	0.090 / 0.085 m
per-scan correction from prior (p99)	0.123 / 0.130 m	0.044 / 0.034 m
Before, z monotonically collapsed around scans ~800–930 (exactly where rotation speed peaks). After, the trajectory stays level and returns near the origin. LiDAR-only was already level, which confirmed the IMU fusion path as the source.
The fix
In Slam::Predict I now:
- rotate on SO(3) with the bias-corrected gyro: R ← R · Exp((gyro − bg)·dt), then re-extract Euler for the public pose;
- gravity-compensate with the bias-corrected accelerometer: R·(a − ba) − g.
Added Slam.PredictIntegratesRotationOnSo3 in test/test_slam.cc, which fails under the old Euler behavior and passes now. ctest is green (the two *_NOT_BUILT moptim test stubs are pre-existing and unrelated).
Remaining risks I deliberately did not change
Still worth addressing if you want full robustness:
- Failed/poor registration is inserted into the map anyway (ImuRegistration::Align returns the prior; run() always addScans) — no fitness/inlier gate.
- Init scans are inserted untransformed while Predict moves state_, so the first tracked frame starts from a drifted prior.
- Search radius mismatch: adjacent_voxels_ = 1 with resolution 0.05–0.1 but max_correspondence_distance = 0.15.
- No robust kernel / outlier rejection on the point-to-plane cost.
- kImuWeight = 0.01 remains a magic number, now that extras_i makes the biases observable.
Want me to tackle the registration-failure gating and init handling next? Those are the next most likely to corrupt the map under aggressive motion.