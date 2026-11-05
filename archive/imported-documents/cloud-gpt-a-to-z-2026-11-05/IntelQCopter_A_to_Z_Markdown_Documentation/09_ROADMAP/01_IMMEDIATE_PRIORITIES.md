# Priority roadmap — immediate engineering work after Nov 5 snapshot

## P0: preserve the golden baseline

Make immutable backups of the successful 4 Nov full sensor source/archive, raw CSV, calibration candidate JSON, and the v2 FC source. Record checksums, actual `0x3D` MAG profile and 921600 UART. Do not overwrite the baseline with a failed FIFO experiment.

## P1: verify FC v2 on real ESP32 with motors OFF

Run PlatformIO real build, upload, log 30 seconds and then a longer 10-minute run. Check boot offset write/readback and residual bias, true 500 Hz loop timing, FIFO lost/overflow, MAG rate, BMP rate, I²C fallback and UART. This is the highest-yield next task.

## P2: verify geometry and estimator

Physically survey sensor orientations, body axes and static acceleration sign. Confirm calibrated gravity at multiple known poses, MAG yaw trend, pressure height at known ruler heights and re-zero chord. Keep motor outputs disabled.

## P3: identify propulsion and safety

Weigh actual assembled drone; physically map motors/ESC; establish independently safe guarded thrust/lag measurement and voltage/current monitoring. Implement reviewed, verified arming, independent cut-off and lost-link behavior in a **separate** hardware-test phase.

## P4: add LiDAR after physical pin validation

Calibrate with independent ruler 25/50/100/200 cm, verify 16-byte frames/checksum and update rate, fit uncertainty, and re-evaluate altitude Kalman under static and dynamic tests. No claiming near-ground hover improvement until observed.

## P5: autonomy and companion PC

Only after primary stabilization is flight-verified; ROS2/YOLO loads must never preempt time-critical FC control. Preserve minimal hardware and clear failure authority.
