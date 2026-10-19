# Modelling and autonomy work log — 2026-10-20

Requested outputs: detailed LaTeX model and six PID/Kalman derivations; ROS 2 C++ companion and YOLO integration; test communication with the C flight core; video and graphs of the strongest measured simulation result.

Capability mapping: physics-simulation (plant/rigid body); isaac-sim-ros2-bridge (ROS topics); isaac-sim-rendering (capture); isaac-sim-remote (live server/health); isaac-sim-orchestrator (foundation gates/evidence).

Known constraints: physical mass, inertia and thrust curves are not measured. RTX 3050 Laptop 4 GB, driver 555.99, Windows 10. WSL is not operational; ROS 2 is not in PATH. Native C/C++ and CPU model tests can be executed independently. No physical actuator outputs will be enabled.

Evidence policy: distinguish Isaac rendering/physics, ROS 2 DDS transport, shared C controller SITL, standalone navigation core, and YOLO inference. Record failures; a CPU video is labelled as such and never substituted for Isaac evidence.

## Completed and observed

- C: host-only DLL ABI; exactly six PID objects; moving altitude target with
  vertical-speed derivative; corrected FLU pitch mixer; Joseph covariance form.
  Fresh barometer observations are corrected once, not repeatedly at500Hz.
- Three nominal candidates and three held-out scenarios pass. Selected scale1.3:
  21.5s route, 0.01199m altitude RMS, 0.30058m prop clearance. Refinement runs
  pass with1/0.5ms physics and unchanged500Hz control/noise.
- C Monte Carlo:500 angle and500 altitude trials, separate matched Gaussian models.
- Bundled ROS2 Jazzy works with isolated NumPy2.2.6 in build/ros_python, explicit
  DLL directories and workspace logs. UDP-only profile avoids denied SHM creation.
  Two same-process Python nodes complete10,751 C-controller round trips. This does
  not prove RP2350 deadlines or compiled ROS C++ wrapper behaviour.
- Actual GPU YOLO11n and OpenCV ONNX CPU reference detections pass. ROS C++ YOLO
  source authored, unbuilt. The route still uses a known static map.
- USD scene authored/parsed:18prims, assumed positive mass/inertia,.34m prop envelope.
- Both Isaac startup attempts fail GPU enumeration; health never passes. No live
  force/sensor/ROS adapter or rendering is validated on this host.

## Lessons retained

The project Python3.10 environment is distinct from standalone Kit Python3.12.
Use compatible NumPy/DLL paths for bundled ROS bindings. Use stamped custom
autonomy messages and explicit frame/velocity conventions. Include propeller
radius in collision clearance. Sample variance is not continuous noise spectral
density. Parser validation is not physics proof. Preserve failed Isaac logs;
CPU replay must stay labelled. The LaTeX WindowsApps resource cannot execute
directly here; a workspace copy of the same compiler can create the PDF.
