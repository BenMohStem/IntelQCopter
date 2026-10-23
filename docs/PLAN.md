# Plan before implementation

23 October 2026. Baseline: previous ESP32 branch commit `c194112`. This is a clean active tree with preserved Git ancestry, not an assertion that older work passed on hardware.

## Acceptance ladder

| Step | Small deliverable | Evidence required before acceptance |
|---|---|---|
| 00-inspection | Wiring/material worksheet, supply and continuity checks | Actual board/module identity; unpowered checks; measured rails; propellers removed, ESCs disconnected |
| 01-board | Boot banner, uptime, reset reason, chip identity; no outputs | Real serial log after boot and repeat boot; correct board identity, stable operation |
| 02-i2c | Bus scan and chip-ID reports | Real ACKs and identity registers for MPU6050/BMP280; address/pin map confirmed |
| 03-imu-baro | Fresh raw IMU samples, stationary mean/std; compensated pressure/temperature | Real stationary and orientation/height-change captures; no read failures; timing and noise reviewed |
| 04-calibration | Stationary gyro bias, six-face accelerometer calibration, flash persistence | Actual six orientations, movement rejection, save/readback/reboot, corrupt/absent-record handling |
| 05-xbox | Gentle stick/trigger mapping, disarm gesture, disconnect detection | Real Xbox reports; direction/centering/latency/reconnect logs; motor pins inactive |
| 06-estimator | Angle/bias and height/vertical-speed estimates | Tilt/height reference measurements, residuals, covariance/noise checks; no invented sensor accuracy |
| 07-controller-bench | 500Hz task, six control loops, mixer and headroom; calculated demands only | Hardware timing/jitter and saturation traces; neutral/recovery/fault behaviour; no ESC output |
| 08-companion | Optional framed UART, heartbeat, mode arbitration | Real serial cable and companion test; standalone works; loss/CRC/stale-command tests |
| 09-simulation | Same control implementation in Gazebo; manual and multi-node navigation | Original simulation logs/video/graphs; actual engine, assumptions and interfaces identified |
| 10-pcb | New schematic, layout, mask and assembled carrier | Known module dimensions, native CAD checks, etched continuity/shorts/rails, repeat earlier affected tests |
| 11-actuators | A separately authorized props-off actuator experiment | **Locked:** current instruction forbids motor output. Requires explicit authorization and a separate procedure |
| 12-flight | Incremental flight acceptance | **Locked:** no flight readiness claimed. Requires preceding acceptances and explicit authorization |

Magnetometer and LiDAR are optional later branches of sensor work. They do not become dependencies of the first MPU6050/BMP280 bench program. Identify actual magnetometer silicon first; a GY-271 label alone is not enough.

## Current iteration

1. Commit this plan and minimal project architecture.
2. Implement a validation ledger that binds acceptance to exact source, hardware configuration and upstream acceptance events. Exercise failure/invalidation with temporary synthetic fixtures only.
3. Implement board/I2C/MPU6050/BMP280 bench diagnostics in one main.cpp. No PID, Xbox, ROS2, NVS or motor driver in this first implementation.
4. Compile and retain the real compiler output. If hardware is not connected, stop at **built, physical test pending**.
5. When the real bench is ready, run 00, then 01, then 02, then 03. Only after reviewing those measurements add calibration in a new commit.

## Offline work and failed steps

```text
Implement A -> compile A -> physical acceptance A -> implement B -> acceptance B
                   |
                   +-> optional offline preparation of B (not accepted)

A fails -> record failure -> block B and descendants -> fix A -> retest A
       -> discard/replace obsolete active B implementation -> retest new B
```

The code can be developed without hardware. What cannot be done without hardware is claiming its physical acceptance. Do not automatically erase a later step's code after a failure: it may be useful diagnostic evidence. Keep it in Git history; remove it from the active build with a descriptive new commit when it relies on an invalid assumption. Never force-reset shared history to hide a failed trial.

A fresh pass for A does not silently restore B's old pass. B must be tested against the new accepted A. Changes to source, bench wiring/configuration or original evidence similarly make old acceptance stale. A simulation pass remains simulation evidence; its parameter assumptions still require measurement.

## Questions resolved later by measurement

Exact ESP32/module revisions; sensor orientation; supply after breakout regulators; I2C pull-ups; mass/CG/inertia; motor identity; thrust/current/lag with9047 props and3S; ESC protocol/range; LiDAR placement; companion platform. The initial programme measures sensors, not these unknown flight parameters.

## Confirmed control scope

The user has the full ESP32-S3 drone assembly, including MPU6050, barometer, magnetometer, LiDAR and companion. Only sensor/calibration diagnostics are authorized in this iteration. The future standalone mode is yaw rate, roll/pitch angle, and fixed altitude plus vertical-speed commands, using PIDs. **No automatic XY brake or position hold.** Zero tilt does not itself cancel horizontal momentum. ROS2 is a later optional extension. The initial host probe found no COM ports; no firmware was uploaded.
