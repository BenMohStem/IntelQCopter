# Current status — ESP32/Xbox

## Implemented and checked

- Flight control, estimation, calibration, Xbox parsing, mode gestures and UART authority stay in the single `firmware/src/flight_controller.c`.
- Stable mode: roll/pitch angle cascades, yaw rate, centred-stick height hold/vertical speed. RT boosts; LT reduces. Down+A/B held five seconds selects stable/navigation. Down+LT held two seconds latches disarm.
- Physical arming remains disabled: motor pins are inputs, and no ESC output driver is connected.
- ESP32-S3 hardware build succeeds: 31,836 bytes static RAM, 590,629 bytes reported program flash. Real heap, task timing and hardware remain unmeasured.
- Latest Espressif QEMU ESP32-S3 replay passes 44 named checks, including raw HID gestures, calibration, stale inputs and CRC/sequence/range-checked UART commands. These are synthetic input tests, not flight physics.
- The active C core also drives native Gazebo 10.5/OGRE2 through simulated IMU/barometer samples at logical 500 Hz. Five virtual Xbox manual flights passed. All ten final navigation layouts also passed, plus an additional PC-map endpoint workflow. Original trials and corrections remain in `artifacts/gazebo-suite/`; see [final evidence](gazebo-results-2026-10-21.md).
- ROS2 C++ drone mission/perception/UART software and a PC endpoint map are implemented. PC supplies endpoints over DDS; the drone plans/replans from RGB YOLO and known-size panel PnP observations.
- Initial Windows DLL mismatch, recording serializer failure, gesture timing gap, occlusion-distance failure and interrupted trials are retained. No failing check or native contact is suppressed.

## Flight station, recordings and analysis

[Current instructions](live-xbox-and-companion.md). The unified station contains native follow/overview/onboard cameras, actual Xbox input monitoring, explicit simulator mode buttons, readiness reasons, preserved-trial reset and a reviewed 2–64-node PC editor. Gentle simulation input uses 5° maximum tilt, ±0.5 m/s vertical command, ±45°/s yaw and a squared stick curve. This is not horizontal position/speed hold; physical firmware gains/limits are unchanged.

The initial human session remained on the ground because no joint five-second mode gesture reached the FC; x=9.6 m was also outside the allowed route boundary. Original rejection/inputs are retained. A later browser-approved five-node mission completed before subsequent manual collisions; the whole session must not be described as contact-free. The reset scientific-notation failure and corrected recovery evidence are preserved.

New recordings target native 15 fps with 500 Hz telemetry. Earlier 5 fps / 50 Hz evidence remains intact. The recording-analysis tool scans each original frame, retains numerical metrics, and provides frame stepping and graph means/min/max. [Original report](../artifacts/reports/station-analysis-20261021/index.html).

ROS2 heading now follows intended path velocity and holds its target near nodes/landing. On one repeatable five-node comparison, total yaw travel fell 57.0% and yaw-rate RMS fell 39.5%; both routes passed. The separate 15 fps route also passed. These are simulated-case results, not hardware confidence bounds. Original failures, collision episodes and baseline sources remain available.

## Evidence categories

QEMU runs the active C source on an emulated Xtensa CPU, without a flight plant.
Gazebo runs the active C source as a host DLL with real native physics/cameras.
ROS2 nodes run as actual independent Windows processes. They simulate separate
PC/drone roles on one machine. Neither result proves physical BLE/I²C/UART/ESC,
measured 500 Hz deadlines, real aircraft stability or separate-computer networking.

The F450 model uses an assumed 1.2 kg and inertia (0.012, 0.012, 0.022) kg·m²,
6 N/motor and 40 ms lag. Sensors/localization/calibration are synthetic. RGB
ranging is specific to known-size, flat-floor blue panel fixtures, including a
known centre-height prior. It does not resolve arbitrary obstacle distance.

## Remaining hardware work

1. Exact-board wiring, sensor axes, I²C, controller pairing/firmware and real report timing/range.
2. Real calibration/noise/temperature data and NVS reboot/power-loss behavior.
3. HMC5883L, lidar orientation/driver, battery voltage/current acquisition and SOC characterization.
4. Complete takeoff mass, COM/inertia, thrust curve, motor delay and reaction torque measurements.
5. Companion hardware/OS, camera/localization source, standard odometry-frame adapter, UART electrical tests and separate-machine DDS.
6. A future explicitly authorized ESC/arming implementation; it remains disabled now.

## Software cleanup

25 Isaac Sim Python packages were uninstalled. Direct removal of remaining Isaac
namespace/staging data and historical Isaac files was blocked by automatic command
policy. See `artifacts/maintenance/2026-10-21/isaac-cleanup.json`. The shared vision
Python environment was retained, including its historical directory name. Base
ROS2/Gazebo installation files were not changed by this task.

Prior project history remains recoverable through `archive/README.md`. Original
logs, failed trials and videos are evidence rather than disposable cache.


## Multi-node fix, authority audit and workshop — 2026-10-21

- Seven-node route reproduced from the user's failed session now completes twice with one PC request, neutral pilot afterward, five intermediate acknowledgements and zero native obstacle contact steps. Final XY errors: 8.14 mm / 16.76 mm. The final repeat took 85.12 simulation seconds. Source/binary hashes, DDS publisher identity and raw CRC/sequence audit are retained.
- Planner start-in-clearance deadlock fixed by conservative outward recovery; hard obstacles remain blocked, invalid goals report a reason, failed plans retry. Deliberate UART loss prevented completion and caused horizontal drift; no position-hold failsafe is claimed.
- Recorded human HID replay in standalone mode used the same FC DLL, no ROS2 and no feedback pilot. It took off and moved; it is not a new human flight or successful landing test. The earlier preflight-only replay remains preserved.
- `station` opens the persistent launcher/settings/recording library on port 8088; `station-stop` gracefully saves. Live C telemetry graphs appear below the current station. Settings are host-simulation experiments applied at next launch only. Defaults restored after verification.
- QEMU: 50/50 checks; both firmware variants and the ROS2 overlay compile. Planner regressions, C tuning validation, profile isolation, native 15fps video decoding and HTTP video seeking passed.
- Engine: native Gazebo 10.5/OGRE2 plus C host DLL for physics trials; separate Xtensa QEMU replay. Model remains assumed (1.2kg, J=.012/.012/.022kg·m², 6N/motor, 40ms lag). Synthetic IMU/barometer/calibration and perfect Gazebo localization remain. Real BLE/I2C/UART/ESC and hardware timing unvalidated; physical arming disabled.

[Original video, graphs and audit](../artifacts/reports/authority-20261021/index.html) · [Settings/code/math guide](control-workbench.html). The 199MB original manual-session CSV is retained locally; its verified lossless gzip and SHA256 manifest are versioned beside it.


## Current manual angle/hold and automatic lifecycle — 2026-10-21

- Recording starts automatically. Home Start opens a new simulation tab; End session stops the flight/recording and closes that script-created tab after encoder completion. Home remains open; manually opened tabs follow browser closure restrictions. Original encoder marker and browser screenshots are retained.
- Manual roll/pitch remain angle requests. Fresh explicit localization enables C FC release-point capture, bounded braking and position hold; manual yaw is reversed once in the Xbox decoder. Without real horizontal localization, IMU/barometer alone cannot hold XY. Physical localization drivers and arming are not enabled.
- Sustained-angle test exposed a real estimator defect: translational acceleration treated as gravity caused angle creep and a failed trial. With fresh localization velocity, manual dynamic gravity corrections are now withheld while gyro prediction/covariance continue. Navigation policy remains separate. Original failed video/logs preserved.
- Final four-second 5° holds measured 5.059° pitch / 5.066° roll. After 14 seconds of release settling, point error ≤3.12cm, speed <0.019m/s, angle <0.024°. Short-step test error ≤5.18cm. These are assumed-model/perfect-localization simulation results, not measured hardware accuracy or instantaneous stopping.
- Final seven-node ROS2/C FC regression completed every intermediate node with zero panel contact steps. QEMU 54/54; both firmware builds pass; host aiding validation/expiry tests pass.

[Current original video, graphs and failures](../artifacts/reports/manual-control-20261021/index.html) · [Updated math/code guide](control-workbench.html).
