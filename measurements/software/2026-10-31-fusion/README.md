# Quaternion fusion preview — 31 October 2026

Adafruit AHRS2.4.0 Mahony IMU algorithm replaces direct gravity-angle output in lessons/attitude-preview/main.cpp. This is a quaternion complementary filter, not Kalman. Existing sensor trims, biases and historical proper rotation remain unchanged; production firmware/main.cpp user edit remains preserved separately. Motors disabled; Xbox not integrated in this preview.

At startup initialize a unit quaternion from gravity roll/pitch, yaw zero (a reference, not a heading observation). Each FIFO sample propagates at nominal0.001s using body gyro converted rad/s to deg/s as the library expects. Kp=0.5, Ki=0 are provisional estimator gains, unrelated to flight PID gains. Accelerometer correction is skipped when |norm(a)-g|>=1.5m/s². This heuristic cannot reject every translational disturbance. Normalize q and reject invalid state. Euler pitch conversion is clamped before asin to prevent floating-point domain errors. Stop on startup acceleration inconsistent with gravity; keep assembly resting when starting.

Quaternion continuity does not make Euler angles unique near vertical pitch or prevent roll wrap at±180degrees. Warnings remain. MPU-only fusion cannot correct absolute yaw drift. No claim of flight accuracy or hard1ms full-controller deadlines. Source change invalidates old hardware computation/timing acceptance.

Build: .\qcopter.ps1 build-attitude-preview
Upload: .\qcopter.ps1 upload-attitude-preview --upload-port COM3
Display: .\qcopter.ps1 view-attitude --port COM3

One short check: startup resting, gentle tilt/return about each physical axis, inspect recovery and existing logged diagnostics. Stop when behavior is understood; no new calibration campaign. Physical signed directions still require reference before motor feedback.

Primary source: https://github.com/adafruit/Adafruit_AHRS/tree/2.4.0 . Actual installed source is checked by the host tests; synthetic cases are not hardware validation. Runtime remains one main.cpp plus Arduino libraries.

Validation: PlatformIO ESP32-S3 build passed (RAM19592bytes, flash320821bytes). Nine native host cases using the actual MCU helper and installed library passed: positive/negative90degree gyro rotations about all three axes, recovery toward a known gravity direction, rejection of nonfinite gyro input, rejection of invalid startup gravity. These synthetic references do not prove physical axis labels, full motion accuracy, ESP timing or vibration tolerance. New source is not uploaded. Existing record-1khz checks target the historical protocol and must not be treated as validation of FUSION_PREVIEW_V1.

