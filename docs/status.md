# Current status — ESP32/Xbox

## Implemented and checked

- Flight control, estimation, calibration, Xbox parsing, mode gestures and UART authority stay in the single `firmware/src/flight_controller.c`.
- Stable mode: roll/pitch angle cascades, yaw rate, centred-stick height hold/vertical speed. RT boosts; LT reduces. Down+A/B held five seconds selects stable/navigation. Down+LT held two seconds latches disarm.
- Physical arming remains disabled: motor pins are inputs, and no ESC output driver is connected.
- ESP32-S3 hardware build succeeds: 31,836 bytes static RAM, 590,573 bytes reported program flash. Real heap, task timing and hardware remain unmeasured.
- Latest Espressif QEMU ESP32-S3 replay passes 39 named checks, including raw HID gestures, calibration, stale inputs and CRC/sequence/range-checked UART commands. These are synthetic input tests, not flight physics.
- The active C core also drives native Gazebo 10.5/OGRE2 through simulated IMU/barometer samples at logical 500 Hz. Five virtual Xbox manual flights passed. Original navigation trials and subsequent corrections remain in `artifacts/gazebo-suite/`; final layout verification is recorded in its reports.
- ROS2 C++ drone mission/perception/UART software and a PC endpoint map are implemented. PC supplies endpoints over DDS; the drone plans/replans from RGB YOLO and known-size panel PnP observations.
- Initial Windows DLL mismatch, recording serializer failure, gesture timing gap, occlusion-distance failure and interrupted trials are retained. No failing check or native contact is suppressed.

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
