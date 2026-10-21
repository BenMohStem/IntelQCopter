# Project documentation

One maintained destination for each task; older entry points link here or to the corresponding guide.

| I want to… | Read |
|---|---|
| Start, fly, plan routes, record and stop | [Run the simulation](run-simulation.md) |
| Change FC/ROS2/model settings and understand who commands the drone | [Control workshop](control-workbench.html) |
| Learn force, F450 geometry, differential equations, transfer functions, six PID gains, Kalman filters and discrete C updates | [Illustrated mathematical guide](current-model-and-fc-integration.html) — [LaTeX source](current-model-and-fc-integration.md) |
| Measure the actual aircraft and sensor uncertainties | [Measurement procedures](../measurements/README.md) |
| Follow current progress and limitations | [Status](status.md) and [factual knowledge overview](../knowledge/current.md) |
| Find firmware/ROS2 code and interfaces | [Code map](code-map.md), [architecture](architecture.md), [protocol](xbox-ros2-protocol.md) |
| Debug timing, authority, faults and saturation | [Debugging guide](debugging-and-flight-readiness.md), [original evidence report](../artifacts/reports/hardening-20261021/index.html) |
| Build the home-etched carrier | [PCB, schematic, artwork and assembly](../hardware/home-etched-fc/README.md) |
| Recover older documents and decisions | [Documentation versions](../archive/documentation/README.md), [versioned knowledge ledger](../knowledge/README.md) |

**Scope:** ESP32-S3 competition branch. Native Gazebo uses the actual C FC compiled on the PC and actual ROS2 C++ nodes, with a provisional plant, synthetic sensors and ideal localization. QEMU is firmware emulation, not a physics engine. Physical motor arming remains disabled.
