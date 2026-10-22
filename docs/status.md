# Current status — 22 October 2026

## Implemented and exercised in software

- ESP32-S3 C-style C++ FC, Xbox input, Gentle angle commands, yaw rate, vertical-speed/hover control, bounded trigger collective headroom and optional position-aided braking.
- Native Gazebo physics with synthetic sensors and ideal localization; actual host-compiled firmware core and ROS2 C++ navigation. Routes support intermediate nodes. Original successful and failed trials remain under `artifacts/`.
- Workshop settings applied on the next launch, automatic 15 fps recording, live plots and session start/end controls.
- Debug counters, host stress checks, ESP32 QEMU replay and ROS2 fault/authority checks: [original report](../artifacts/reports/hardening-20261021/index.html).
- Home-etched ESP32 carrier CAD: zero native DRC/ERC/parity/unrouted issues. This is CAD evidence only: [PCB package](../hardware/home-etched-fc/README.md).

Latest implementation: [one-file FC, optional ROS2 and profiling evidence](../artifacts/validation/single-cpp-optional-ros-20261021/README.md). The ESP32 GND pads now connect directly to the common copper fill; W22 was removed. [Annotated mask continuity proof](../artifacts/hardware/direct-common-ground-20261021/ground-connection-proof.png).

Latest PCB: [solid GND pads, round drill openings and full power/mask audit](../artifacts/hardware/solid-ground-power-audit-20261021/README.md). Thirteen insulated links are required, including lidar5V W23. [Key source shelf](../important_sources/README.md).

Latest sensor configuration: [MPU6050 + BMP280 only](esp32-mpu6050-bmp280.md), with auto-detected I2C addresses, required-device configuration checks and explicit absent optional-sensor logs. [Both Arduino hardware builds](../artifacts/build/mpu6050-bmp280-20261022/README.md) compile; physical operation remains unmeasured.

## Still unvalidated

- Physical motor outputs are disabled. ESC pulse driver, real timing, motor order and sensor wiring need bench work.
- Full aircraft mass, inertia, thrust curves, motor lag, sensor noise and battery measurements are not identified. Mathematical gains remain provisional.
- Real horizontal localization is not implemented. IMU + barometer do not provide drift-free XY hold. Simulated known-panel distance estimation is not general RGB depth.
- PCB module/frame fit and supply behavior remain unmeasured; current sensor, calibrated ADC driver, magnetometer integration and purchased lidar connector/revision and physical interface validation remain open; reference F2 Mini supply is regulated5V.

## Where to continue

[Documentation map](README.md) · [Current factual knowledge](../knowledge/current.md) · [Launch](run-simulation.md) · [Math](current-model-and-fc-integration.html) · [Measurement worksheets](../measurements/README.md).

The RP2350/RP2040 radio architecture remains on main. Earlier chronological status text is [retained in the documentation archive](../archive/documentation/README.md).
