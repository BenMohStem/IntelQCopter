# Documentation index

Start with the current system map and project plan, then open the subsystem guide
for the work in hand. Original measurements and failures are indexed separately
under [test evidence](../artifacts/test-runs/README.md).

## System and flight control

- [ESP32-S3 competition-readiness plan](esp32-competition-readiness-plan-2026-11-06.md)
  — near-term scope, current evidence, conflict resolutions, and validation gates.
- [Current model, gain calculations and flight-controller integration](current-model-and-fc-integration.md)
  — equations, transfer functions, stability, Kalman gains, and the actual DDS/Gazebo path.
- [Flight-controller C walkthrough](flight-controller-line-by-line.md) — source, units,
  equations, and unfinished hardware connections.
- [RP2350/RP2040 code walkthrough](reading-the-code.md) — flight scaffold, input scanner,
  radio packet, and hardware gaps.
- [Controller input map](controller-inputs.md) — proposed pin map, Mode 2 stick semantics,
  buttons, and protocol.
- [ROS 2 companion setup](companion-computer-setup.md) and
  [companion package overview](../companion/README.md) — ROS interfaces, planner, and
  YOLO ONNX node.
- [Mass and Kalman uncertainty](model-uncertainty.md) — assumptions, distributions,
  metrics, and limits.
- [F450 model sources](f450-model-sources.md) — source and assumptions behind the aircraft
  model.

## Sensors, calibration, and hardware

- [TOFSense-F2 Mini geometry and altitude fusion](lidar-geometry-and-altitude-fusion-2026-11-06.md)
  — reported F450 mount, lever-arm equation, terrain limits, and estimator gates.
- [Sensor calibration](sensor-calibration.md) — implemented fits, capture procedure,
  uncertainty, and hardware gaps.
- [ESP32-S3 calibration GUI](esp32-calibration-gui.md) — guided capture procedure and
  QMC evidence limits.
- [ESP32 bench validation log](../hardware/bench_calibration/esp32s3_qmc5883p_100hz/VALIDATION_LOG.md)
  — chronological preflights, failed trials, and bench evidence.
- [Calibration flash storage](calibration-flash.md) — saved records and firmware
  integration.
- [Magnetometer triage](magnetometer-triage-2026-11-03.md) — QMC/HMC identity and
  read-path investigation.
- [Motor limits](motor-limits-2026-10-28.md) — current command caps and arming policy.
- [Flight-controller PCB notes](../hardware/flight_controller_pcb/README.md) — draft board
  state and unrouted nets.
- [Missing information and current problems](missing-information-and-problems.md) —
  hardware, autonomy, and measurement priorities.

## Simulation and reports

- [Current native Gazebo workflow](../sim/gazebo/README.md) — engine setup and launch
  instructions.
- [Recorded simulation, graphs, and data](../artifacts/simulation/2026-10-20-missions/index.html)
  — CPU plant and ROS DDS route; Isaac startup failure is recorded separately.
- [Uncertainty visualizations](../artifacts/visualizations/uncertainty-dashboard.html) —
  replay and plots from saved Monte Carlo data.
- [Mathematical model report (PDF)](reports/IntelQcopter-model-guide.pdf) and
  [LaTeX source](modelisation/main.tex) — model, PID gains, Kalman derivations, and
  simulation results.
- [Historical Isaac integration](../archive/isaac/README.md) — retired engine work
  retained for provenance.
- [Windows ROS 2 Jazzy setup](windows-jazzy-setup.md) — machine setup notes and current
  constraints.

## Project organization and shared notes

- [Architecture and migration map](architecture.md) — repository layout and file lifecycle.
- [Current project plan](project-plan.md) — cleanup, calibration, readability, and review phases.
- [Workspace cleanup and local archive map](workspace-cleanup-2026-11-07.md) — canonical checkout, removed duplicate worktrees, local-only handoffs, and GPT_TMP cache inventory.
- [Local build inventory](build-directory-inventory-2026-11-05.md) — ignored mixed workspace
  contents; do not bulk-delete.
- [Shared AI thought base](thoughts/README.md) — current handoff, canonical database, and
  collaboration rules.
- Run `qcopter.ps1 help` from the repository root for current project commands.
