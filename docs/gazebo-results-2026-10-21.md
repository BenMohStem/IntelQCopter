# Native Gazebo results — 21 October 2026

## Verified final cases

| Case | Passed | Mean landing error | Across-case sample SD | Maximum |
|---|---:|---:|---:|---:|
| manual | 5/5 | 9.49 cm | 6.28 cm | 14.11 cm |
| navigation | 10/10 | 1.53 cm | 0.51 cm | 2.81 cm |

No native panel contacts were recorded in the final 15 cases. These are selected scenario results, not a physical reliability probability or sensor-noise confidence interval. Earlier failures remain linked in the report.

[Open all videos, graphs and original logs](../artifacts/reports/gazebo-verified-20261021/index.html) · [Machine results](../artifacts/reports/gazebo-verified-20261021/summary.json) · [File/CRC/video audit](../artifacts/reports/gazebo-verified-20261021/evidence-audit.json)

![Ten navigation trajectories](../artifacts/reports/gazebo-verified-20261021/navigation-trajectories.png)

## Command path

- Five manual flights: raw simulated Xbox HID reports only, zero companion command packets and zero PC mission requests. Virtual humans use the simulated pose/velocity HUD, not actual recorded human inputs.
- Ten navigation flights: one accepted PC endpoint request per case; independent C++ mission/YOLO/PnP nodes generate bounded UART setpoints for the active C controller. Unknown panel positions are inferred from native RGB, not passed as a hidden map.
- Extra PC map workflow: the HTTP endpoint tool sent a custom finish `(8.9, 1.0, 0.082)` and cruise height `1.3 m` through the real ROS2 service. The virtual Xbox sent no automatic mission request. It completed and disarmed. An initial start-point mismatch was rejected and retained in the HTTP request log.

Operator landing error: 1.88 cm. [Original operator video](../artifacts/gazebo-suite/pc-map-operator-20261021-env07-nav/gazebo.mp4) · [Requests and replies](../artifacts/gazebo-suite/pc-map-operator-20261021-env07-nav/pc-map-missions.jsonl).

## Scope and assumptions

Native Gazebo 10.5/OGRE2, 500 Hz logical C host control, actual ROS2 DDS/C++ processes. Separate Espressif QEMU replay passes 39 named checks; the flight plant does not run on the emulated CPU. Physical motor outputs remain disabled.

The assumed model is 1.2 kg, J=(0.012,0.012,0.022) kg·m², 450 mm motor diagonal, 6 N/motor, 40 ms lag, 0.12 N·s/m drag. Synthetic noise is 0.001 g accelerometer, 0.02 deg/s gyro, 0.008 m pressure-equivalent height. Localization is Gazebo odometry. None of these values is a physical identification result.

RGB ranging depends on known-size blue panel fixtures (inner face 1.09 × 1.69 m) standing on a flat floor with centre height 0.90 ±0.20 m. It does not establish depth to arbitrary objects. Lidar, real localization, voltage/current sensing, electrical UART/BLE, physical timing and flight are not validated. Battery depletion is not modeled in these flights.

## Reproduce

See [launch instructions and PC map mode](../simulation/gazebo/README.md). Base ROS2 installation files were not changed. All 24 pre-existing Pico evidence files retained their original hashes.
