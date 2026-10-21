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
