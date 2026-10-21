# Documentation index

- [Current model, gain calculations and FC integration](current-model-and-fc-integration.md) — stationary gyro averaging, nonlinear equations, transfer functions, full-order stability, periodic Kalman gains, actual DDS/Gazebo path and future wired bridge.
- [Every flight-controller C line explained](flight-controller-line-by-line.md) — exact source with units, equations and unfinished hardware connections.
- [Detailed mathematical PDF](modelisation/main.pdf) and [LaTeX source](modelisation/main.tex) — modelling, six PID gains, Kalman derivations and recorded simulation results.
- [Simulation video, graphs and original data](../artifacts/simulation/2026-10-20-missions/index.html) — CPU flight model and ROS DDS route; Isaac startup failure recorded separately.
- [ROS C++ companion](../companion/README.md) — common planner, ROS interfaces and YOLO ONNX node; implementation and test boundaries.

- [Sensor calibration](sensor-calibration.md) — implemented fits, capture procedure, uncertainty, and hardware gaps.
- [Controller code walkthrough](reading-the-code.md) — explanation of the RP2350 flight scaffold, RP2040 input scanner, shared radio packet format, and current hardware gaps.
- [Hand-controller input map](controller-inputs.md) — proposed pin map, Mode 2 stick semantics, buttons, and protocol details.
- [Uncertainty visualizations](../artifacts/visualizations/uncertainty-dashboard.html) — interactive replay and static SVG/PNG plots generated from the saved Monte Carlo CSVs.
- [Mass and Kalman uncertainty report](model-uncertainty.md) — assumptions, distributions, metrics, and limitations.
- [PCB notes](../hardware/flight_controller_pcb/README.md) — draft board state and unrouted nets.
- [Current Gazebo workflow](../sim/gazebo/README.md) — native engine setup and current launch instructions.
- [Historical Isaac integration](../archive/isaac/README.md) — retained after Isaac was removed.
- [Evidence index](../artifacts/test-runs/README.md) — logs, data, rerun instructions, and proof limits.

- [Missing information and current problems](missing-information-and-problems.md) — priorities and hardware/autonomy limits.

## Repository organization

[Architecture, cache lifecycle and migration map](architecture.md). Run `qcopter.ps1 help` from the root.
