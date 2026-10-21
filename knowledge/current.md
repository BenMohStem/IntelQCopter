# Current project knowledge

Generated from the validated append-only [ledger](project.json). Edit the ledger through a new revision, then regenerate this view. No transcript or private reasoning is stored.

Ledger SHA-256: `b077a9489fc1e898aa060449a7d6cde0d9aac0ec2dc38b06ac30bc5a9c30128a`.

47 facts · 77 revisions · 83 sources.

Status counts: assumed: 1, confirmed: 6, historical: 1, implemented: 18, measured: 15, pending: 6.

[Documentation map](../docs/README.md) · [Math](../docs/current-model-and-fc-integration.html) · [PCB](../hardware/home-etched-fc/README.md) · [Search and history commands](README.md)

## Architecture

### branches

**confirmed · evidence: requirement · esp32-xbox · revision 1**

Main retains RP2350/RP2040. This branch uses an ESP32-S3 on the drone and an Xbox Series X/S controller.

Sources: `source-002` (User request and clarification, 2026-10-20).

### language

**implemented · evidence: source · esp32-xbox · revision 1**

Flight math/calibration in one C file; minimal C++ adapter required by existing Arduino/Xbox/sensor APIs.

Sources: [source-010](../firmware/src/flight_controller.c), [source-007](../firmware/src/esp32_platform.cpp).

## Battery

### battery_measurement

**pending · evidence: none · esp32-xbox · revision 1**

No actual pack voltage/current acquisition or OCV/capacity characterization. An Xbox controller's reported battery level is not aircraft SOC.

Sources: [source-014](../docs/status.md).

### battery_policy

**confirmed · evidence: requirement · esp32-xbox · revision 1**

Warn at 35 percent estimated charge; latch disarm at 20 percent regardless of height. Do not automatically land.

Sources: `source-013` (User battery-policy instruction).

## Build

### build

**measured · evidence: build · esp32-xbox · revision 3**

ESP32-S3 hardware target compiles with 31836 bytes static RAM and 590573 bytes reported program flash. This includes queued raw HID/UART and separate BLE/peripheral tasks; no physical execution.

Sources: [source-build-20261021](../artifacts/validation/2026-10-21/hardware-controller-build-final.log).

## Calibration

### calibration

**implemented · evidence: source · esp32-xbox · revision 1**

500-sample stationary gyro verification; six 200-sample accelerometer faces with geometry/noise gates; no real coefficients captured.

Sources: [source-010](../firmware/src/flight_controller.c).

### nvs

**implemented · evidence: source · esp32-xbox · revision 1**

160-byte versioned CRC/profile/board-bound IMU record in Preferences/NVS. Write spacing/readback and bounded retries. No hardware persistence proof.

Sources: [source-011](../docs/setup.md).

## Companion

### autonomy

**implemented · evidence: source · esp32-xbox · revision 2**

Portable ROS2 C++ drone role accepts PC start/finish requests only in Xbox-selected navigation mode, then runs mission phases/A* from RGB YOLO/PnP observations. Host simulations use Gazebo local odometry. Pi/Jetson, VIO and electrical UART remain unvalidated.

Sources: [source-companion](../companion/ros2_ws/src/qcopter_autonomy/src/pilot_mission_node.cpp), [source-protocol-v1](../docs/xbox-ros2-protocol.md).

### companion_protocol

**implemented · evidence: source · esp32-xbox · revision 1**

Version-1 UART uses QN 28-byte bounded/CRC/sequence-checked setpoints and QS 16-byte FC mode/freshness/disarm telemetry. ROS2 UART bridge supports Windows/POSIX but has no hardware transport proof.

Sources: [source-protocol-v1](../docs/xbox-ros2-protocol.md), [source-qemu39](../artifacts/emulation/final-39-checks-2026-10-21/summary.json).

### ground_pc_map

**measured · evidence: simulated · esp32-xbox · revision 4**

Reviewed PC route editor accepts 2 to 64 nodes, with first=start and last=finish, node editing/reordering, JSON save/load and explicit approval. Browser-approved five-node mission visited all intermediate nodes and landed before subsequent manual collision. Bounds errors now name the invalid node; explicit scene reset can place the vehicle at the first node.

Sources: [source-station-guide](../docs/live-xbox-and-companion.md), [source-station-browser](../artifacts/maintenance/station-ux2-20261021/browser-in-flight.png), [source-station-recovery](../artifacts/maintenance/station-ux2-20261021/recovery-check.json).

### path_heading_hold

**measured · evidence: simulated · esp32-xbox · revision 1**

ROS2 uses intended path velocity for yaw and holds heading near waypoints/landing. Same five-node before/after simulated cases passed; yaw travel decreased 57.03% and yaw-rate RMS 39.53%. This single-case comparison does not characterize hardware or all layouts. A separate 15fps five-node run passed.

Sources: [source-station-analysis](../artifacts/reports/station-analysis-20261021/summary.json), [source-station-checks](../artifacts/maintenance/station-ux2-20261021/checks.json).

## Control

### mode

**implemented · evidence: source · esp32-xbox · revision 1**

Six C controller objects: roll/pitch outer-angle P and inner-rate PI, yaw-rate PI, altitude PID. All gains remain provisional.

Sources: [source-010](../firmware/src/flight_controller.c).

### timing

**measured · evidence: simulated · esp32-xbox · revision 3**

Added bounded queues, C debug levels0..3, rejection/allocator/filter diagnostics, adapter execution histogram and 64-entry50Hz trace. ROS2 steady-wall profiling and latest-frame camera queue are observable. Final host timing is PC ctypes overhead, not ESP32 deadlines; YOLO mean80ms/peak436ms identifies a perception bottleneck. Actual500Hz behavior under BLE/I2C/UART/NVS still requires hardware.

Sources: [source-hardening-final-report](../artifacts/reports/hardening-20261021/summary.json), [source-hardening-guide](../docs/debugging-and-flight-readiness.md).

## Documentation

### documentation_map

**implemented · evidence: source · esp32-xbox · revision 1**

Current documentation has a task index, consolidated launch guide, operational settings guide and expanded illustrated mathematical guide (braking, ideal poles, stopping distance, Gentle shaping, trigger headroom, allocator and filter limits). Earlier document blobs are indexed by immutable Git commit; original trial artifacts remain in place. Current knowledge overview is generated from the validated ledger.

Sources: [source-doc-consolidation](../docs/README.md).

## Emulation

### emulation

**measured · evidence: simulated · esp32-xbox · revision 6**

Final hardened C code passed63 checks on Espressif ESP32-S3 QEMU with original UART logs/hashes. Hardware standard/standalone/debug builds compile. QEMU synthetic replay is not physics, BLE/I2C/UART-device or500Hz timing validation.

Sources: [source-hardening-final-qemu](../artifacts/emulation/hardening-final-20261021/summary.json).

## Estimation

### estimator_limit

**pending · evidence: none · esp32-xbox · revision 1**

Independent angle/bias filters approximate Euler rates; gravity removal uses the previous estimate. Full quaternion/heading fusion remains future work.

Sources: [source-010](../firmware/src/flight_controller.c), [source-014](../docs/status.md).

### estimator_reset

**measured · evidence: emulated · esp32-xbox · revision 1**

fc_init resets angle/bias and height/velocity estimator state and covariance defaults; a reinitialization check passes in the QEMU replay.

Sources: [qemu-replay](../artifacts/emulation/verified-replay/summary.json).

## Evidence

### native_recording_analysis

**measured · evidence: simulated · esp32-xbox · revision 1**

New native cameras/video target 15fps and telemetry is retained at 500Hz; earlier 5fps/50Hz files remain intact. The analysis tool scans every native frame, enables frame stepping, exports graph means/min/max and evaluates navigation separately from later manual collisions. Off-origin reset and a ROS-free 15fps recording were verified.

Sources: [source-station-analysis](../artifacts/reports/station-analysis-20261021/summary.json), [source-station-recovery](../artifacts/maintenance/station-ux2-20261021/recovery-check.json).

## Firmware

### standalone_companion

**measured · evidence: simulated · esp32-xbox · revision 2**

The standard companion-capable FC runs Stable without ROS2/UART, receives optional companion data without granting Stable authority, and retains pilot gesture mode selection. Regressions cover optional enable/disable and actual ROS2 stale-status/pose and Stable cancellation. UART bridge has bounded partial transmit, expiry and configured-port retry; real serial/Linux behavior is unvalidated.

Sources: [source-hardening-final-host](../artifacts/validation/hardening-final-20261021/summary.json), [source-hardening-final-ros](../artifacts/validation/hardening-ros-final-20261021/summary.json), [source-hardening-guide](../docs/debugging-and-flight-readiness.md).

## Hardware

### competition_pcb

**implemented · evidence: build · esp32-xbox · revision 1**

100x100mm ESP32-S3-DevKitC-1 reference carrier with bottom copper, 13 insulated links, 1mm signals, 1.9mm socket pads and 3.2mm external pads. Native KiCad 10.0.6 reports zero DRC/ERC/unrouted/parity issues; independent comparison matches 35 connected nets. Module/frame fit and electrical/physical flight behavior remain unvalidated. No motor-current path or physical ESC output enable.

Sources: `source-competition-pcb-request` (User confirmed ESP32-S3 + Xbox sensors-only competition carrier, single bottom copper with wire jumpers, internet reference modules; 2026-10-21), [source-pcb-native-cad](../artifacts/hardware/home-etch-20261021/verification-summary.json), [source-pcb-assembly](../hardware/home-etched-fc/README.md).

### pcb_measurement_pending

**pending · evidence: source · esp32-xbox · revision 1**

Battery divider and filtered GPIO4/GPIO5 voltage/current provisions are CAD only. Current sensor, calibrated ADC driver, power-off input protection, exact lidar supply, magnetometer driver and purchased module fit remain open. Companion UART is optional; no onboard LoRa/CC1101 in this competition carrier. This is not a hardware BMS.

Sources: [source-pcb-assembly](../hardware/home-etched-fc/README.md).

### target

**confirmed · evidence: requirement · esp32-xbox · revision 1**

Use the esp32-s3-devkitc-1 target from the original Drone PlatformIO project; exact physical board/wiring still needs checking.

Sources: [source-003](../archive/reference-drone/platformio.ini), `source-004` (user clarification).

## Input

### callback_snapshot

**implemented · evidence: source · esp32-xbox · revision 2**

BLE callback queues raw 16-byte Xbox reports and actual receive timestamps. The C control task owns decoding, hold timers and authority; companion frames also carry receive timestamps across the queue.

Sources: [source-007](../firmware/src/esp32_platform.cpp), [source-protocol-v1](../docs/xbox-ros2-protocol.md).

### controls

**confirmed · evidence: requirement · esp32-xbox · revision 2**

Mode 2 sticks retained. RT boosts climb/collective allowance; LT reduces it. Down+A 5s selects stable, Down+B 5s navigation; Down+LT >=90% 2s latches disarm.

Sources: `source-controls-20261021` (User Xbox mode/trigger and simulation requirements, 2026-10-21), [source-protocol-v1](../docs/xbox-ros2-protocol.md).

### gestures

**measured · evidence: emulated · esp32-xbox · revision 2**

39-check QEMU replay verifies full five-second mode holds, report-gap restart, once-per-hold action, two-second LT disarm/latch, malformed HID and UART validation. Old LB/RB takeoff/land logging was replaced.

Sources: [source-qemu39](../artifacts/emulation/final-39-checks-2026-10-21/summary.json), [source-protocol-v1](../docs/xbox-ros2-protocol.md).

### library

**implemented · evidence: source · esp32-xbox · revision 1**

Use XboxSeriesXControllerESP32_asukiaaa 1.1.3 and NimBLE 2.5.1, with a guarded raw-report observer on the downloaded library.

Sources: [source-005](../platformio.ini), [source-006](../tools/platformio/patch_xbox.py).

### watchdog

**implemented · evidence: source · esp32-xbox · revision 2**

Pilot expires after 250 ms; a report gap >100 ms restarts gesture holds. Navigation companion expires after 150 ms and requests neutral angle/rate/vertical command while pilot remains fresh. Thresholds require physical latency/report-rate measurements.

Sources: [source-protocol-v1](../docs/xbox-ros2-protocol.md).

## Model

### airframe

**confirmed · evidence: requirement · airframe · revision 1**

F450/Q450 frame family, 1400KV orange motors of unconfirmed exact model, 9047 props, 40A ESCs, 3S 4200mAh 120C LiPo label.

Sources: `source-016` (User hardware description).

### identify

**pending · evidence: none · airframe · revision 1**

Measure complete mass/COM/inertia, thrust-command-voltage curve, motor delay, reaction torque, noise/drift and actual current/capacity.

Sources: [source-019](https://github.com/BenMohStem/IntelQCopter/blob/8432cf33300fc1f92aeab97f01548aac2b1d2e48/archive/rp2350/measurements/README.md).

### lti_model

**implemented · evidence: source · airframe · revision 1**

A 16-state, four-input F450 hover LTI plant includes 450 mm nominal diagonal, 159.099 mm roll/pitch lever, motor lag and translational drag. Physical parameters are assumed; numerical continuous and exact held-input plant matrices are generated.

Sources: [lti-guide](../docs/current-model-and-fc-integration.md), [lti-data](../docs/assets/lti/calculations.json).

### mass

**assumed · evidence: none · airframe · revision 1**

1.2kg flight scenario and 0.740kg component URDF are separate assumptions, neither measured. Approximate battery mass 150g is unverified.

Sources: `source-017` (User uncertainty), `source-018` (archived model papers).

### sampled_stability

**measured · evidence: simulated · airframe · revision 1**

For the documented assumed plant and ideal state feedback, rounded C gains give stable continuous and 2 ms sampled loops. Largest sampled pole magnitudes: roll/pitch 0.994932, yaw 0.993840, height 0.998631. This excludes actual delay, estimator dynamics and saturation.

Sources: [lti-data](../docs/assets/lti/calculations.json), [lti-guide](../docs/current-model-and-fc-integration.md).

## Organization

### archive

**implemented · evidence: source · esp32-xbox · revision 3**

The duplicate RP2350 snapshot is stored locally at .cache/archive/rp2350 with a versioned SHA-256 inventory and immutable Git recovery reference. Active source remains in the main branch. Tools are grouped by purpose, emulation is under simulation/emulation, and both worktrees have a root command catalog.

Sources: [code-map](../docs/code-map.md).

### knowledge_v2

**implemented · evidence: source · esp32-xbox · revision 1**

Schema 2 stores stable facts, append-only dated revisions, typed sources, evidence levels and dependency relations. SQLite FTS5 is a disposable cache; validation, hash freshness, build locking and atomic replacement protect integrity. Nine isolated failure-containment/search checks pass.

Sources: [knowledge-design](../knowledge/README.md), [knowledge-checks](../artifacts/2026-10-21/knowledge-checks.log).

## Outputs

### arming

**confirmed · evidence: requirement · esp32-xbox · revision 1**

Keep physical arming disabled. Motor pins remain inputs; no servo/ESC output driver is included.

Sources: `source-012` (User retained instruction), [source-007](../firmware/src/esp32_platform.cpp).

## Perception

### fixture_distance_prior

**implemented · evidence: source · esp32-xbox · revision 1**

Known-size blue panel PnP uses synchronized RGB/pose and rejects map centre height outside 0.90 +/-0.20 m on the controlled flat-floor fixtures. This rejects observed occlusion ghosts but is not arbitrary-object monocular depth.

Sources: [source-fixture-prior](../companion/ros2_ws/src/qcopter_autonomy/src/panel_pose_node.cpp).

## Provenance

### source_review

**historical · evidence: none · historical · revision 1**

Original Drone sketch used direct RT throttle, five PID objects, 50Hz ESC servo output, fixed dt and a Boolean tick flag; it was reviewed, not flown.

Sources: [source-020](../archive/reference-drone/src/main.cpp), [source-021](../docs/drone-review.md).

## Sensors

### altitude_sensor

**implemented · evidence: source · esp32-xbox · revision 1**

BMP280 factory compensation with 100-reading session ground reference; pressure-age checking. Actual correlated noise and delay remain unmeasured.

Sources: [source-007](../firmware/src/esp32_platform.cpp), [source-010](../firmware/src/flight_controller.c).

### extra_sensors

**pending · evidence: none · esp32-xbox · revision 1**

HMC5883L, TOFSense F2 mini and battery ADC/current drivers are not integrated on this branch; lidar mounting choice remains open.

Sources: `source-015` (User sensor requirements), [source-014](../docs/status.md).

### imu

**implemented · evidence: source · esp32-xbox · revision 1**

MPU6050 at 0x68, 8g/500dps, 42Hz DLPF, nominal 500Hz sampling; data-ready and complete burst checks. Mount transform remains unverified identity.

Sources: [source-007](../firmware/src/esp32_platform.cpp).

## Simulation

### control_workshop

**implemented · evidence: source · esp32-xbox · revision 3**

GitHub README now gives the configured Windows simulation quick start via qcopter.ps1 station, manual mode/takeoff, 2..64-node route approval, automatic recordings/end-tab behavior and build-host versus hardware-build distinction. New docs/run-simulation.md documents another-PC prerequisites, ignored config paths, missing external ONNX model, hardcoded ROS2 wrappers, direct launch, saved evidence and troubleshooting. Older Gentle-only simulator/20-percent trigger text is corrected. This is documentation checked against project source, not a new flight test.

Sources: [source-github-simulation-launch-guide](../docs/run-simulation.md).

### gentle_simulator

**measured · evidence: simulated · esp32-xbox · revision 2**

Gentle is now also the physical C firmware default: squared axes, 5deg roll/pitch, .5m/s base vertical and45dps yaw. Simulation explicitly shapes once and tests C/UI equivalence. RT/LT retain full travel; allocator collective ceiling ranges .5..8 with neutral .65. 1800 microseconds is only a provisional normalized pulse equivalent; individual differential motor corrections may use the remaining .2 range. Physical outputs disabled.

Sources: [source-hardening-final-host](../artifacts/validation/hardening-final-20261021/summary.json), [source-hardening-guide](../docs/debugging-and-flight-readiness.md).

### isaac

**pending · evidence: source · historical · revision 2**

25 Isaac Sim Python packages were uninstalled. Residual Isaac data/staging folders and historical Isaac project files remain because direct deletion was blocked by automatic command policy. Shared vision Python and TensorFlow/CUDA settings were retained.

Sources: [source-cleanup-20261021](../artifacts/maintenance/2026-10-21/isaac-cleanup.json).

### live_xbox

**measured · evidence: simulated · esp32-xbox · revision 2**

Actual Xbox stick input produced native Gazebo flight. Initial joint-mode gesture was absent, so mode-selection buttons now explicitly emit labelled PC simulator gestures through the C parser. The browser displays native follow/overview/onboard cameras and readiness. A later manual collision is retained, not counted as a successful session.

Sources: [source-station-browser](../artifacts/maintenance/station-ux2-20261021/browser-in-flight.png), [source-station-checks](../artifacts/maintenance/station-ux2-20261021/checks.json).

### manual_authority

**measured · evidence: simulated · esp32-xbox · revision 2**

Manual roll/pitch still command angle. With fresh explicit horizontal localization, the C FC captures the release point, brakes with bounded counter-tilt then holds XY; yaw decoding is reversed. Sustained-angle testing found false accelerometer gravity correction during translation; skipping that correction during measured manual acceleration while preserving gyro prediction/covariance changed 4-second 5deg requests from failed means 12.26/14.70deg to 5.059/5.066deg. Final long test settled within 3.12cm with <0.019m/s residual speed; short test within 5.18cm. No-localization control failed position hold as expected. This uses perfect simulated position/velocity/yaw as a sensor input; no real localization driver, IMU-only horizontal hold or physical flight validation is claimed. Six existing PID loops plus two optional outer XY PD corrections remain in the single C runtime. Navigation regression completed all nodes with no panel contacts.

Sources: [source-manual-control-final](../artifacts/reports/manual-control-20261021/summary.json), [source-manual-control-before](../artifacts/gazebo-suite/manual-long-angle-before-20261021/result.json), [source-manual-aiding-final](../artifacts/maintenance/manual-hold-20261021/manual-final-aiding-checks.log), [source-manual-nav-final](../artifacts/maintenance/manual-hold-20261021/navigation-final-check.json).

### navigation_authority

**measured · evidence: simulated · esp32-xbox · revision 2**

Final native Gazebo C/ROS2 route completed89.04 simulated seconds, intermediate indices1..5 and zero panel contacts. Original wire audit shows onePC route, neutral HID thereafter, sole onboard companion publisher, valid CRC/finite setpoints and noPC pose/image subscription. Dropping companion frames after25s prevents completion and FC marks link stale. Ideal localization/known-panel PnP/synthetic sensors remain explicit.

Sources: [source-hardening-final-report](../artifacts/reports/hardening-20261021/summary.json).

### simulation

**measured · evidence: simulated · esp32-xbox · revision 4**

Final native Gazebo set passed 5/5 raw-Xbox manual flights and 10/10 distinct navigation layouts with no native panel contacts. Mean endpoint errors are 0.094864 m manual and 0.015281 m navigation. Active C host core, synthetic sensors/assumptions and simulated local odometry only; earlier failures retained.

Sources: [source-final-gazebo](../artifacts/reports/gazebo-verified-20261021/summary.json), [source-final-audit](../artifacts/reports/gazebo-verified-20261021/evidence-audit.json).
