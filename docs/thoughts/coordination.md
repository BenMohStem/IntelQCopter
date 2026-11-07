# QCopter AI coordination

Updated: 2026-11-07 (Africa/Lagos)

Read in this order: `AGENTS.md`, this summary, `thought-database.md`, then the
relevant subsystem README and `VALIDATION_LOG.md`.

## Current cleanup update — 2026-11-07

- The current user request is project cleanup and organization. The latest
  committed GUI change is `30d1c41`: the GUI, decoder, session writer, plots and
  Tcl/Tk setup are in
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/tools/live_calibration_gui.py`;
  the PowerShell launcher calls it directly. No firmware behavior or physical
  hardware was changed for that milestone.
- The GUI has 12 IMU/MAG/BMP phases. LiDAR calibration and geometry stages are
  deferred until the later hardware step. Do not follow the older readiness-plan
  paragraphs that describe stages 13–16 as active GUI behavior.
- The user reported a Tkinter callback failure after analysis completion. The
  source fix removes the stale reference to a nonexistent MAG-quality button;
  chart scales now use the displayed time window and unit-aware minimum spans.
  This source change has not been run in the GUI.
- The Nov 6 handoff reported a calibration session in progress. Its present
  completion state is unknown. Treat COM3 and any live capture as user-owned;
  do not open the port or inspect changing capture files without a fresh user
  update.
- The relative competition countdown is stale. Its outcome/date has not been
  confirmed here; don't present the old countdown as a current schedule.

## Current state

- Branch: `feature/esp32s3-competition-fc` (isolated ESP32-S3 candidate).
- Latest DSH software commit before this docs-only handoff: `a7e464d` (`Fix GUI progress bar and latency; QMC raw/37.5 uT`).
- The user-supplied competition specification is translated into
  `docs/esp32-competition-readiness-plan-2026-11-06.md`. It is a design request,
  not validation evidence; current motor and hardware restrictions still apply.
- As of the Nov 6 handoff, the user reported that the final 12-phase IMU/MAG/BMP
  calibration was running with LiDAR excluded. Current run status is unknown.
  Treat COM3 and active capture files as user-owned. Do not open the port,
  restart the GUI, or inspect changing capture files until the user confirms the
  run is complete. Completion requires all 12 phases at target and no skipped
  or partial stage attempts.
- DSH's latest shared ESP32 calibration commit is `a7e464d`. It adds I²C read
  duration fields in `src/main.cpp`; those values are not yet serialized into the
  GUI's v4 telemetry fields and are not per-sensor sample age. Review the source
  before claiming that latency is measured. The duplicate Tk idle refresh in the
  GUI is harmless but redundant; remove one on a later UI edit.
- ESP32-S3 `src/main.cpp` is calibration/telemetry firmware, not the Pico flight
  controller. Keep the sensor capture separate from
  `firmware/flight_controller/flight_controller.c` work.
- The Cloud GPT A-to-Z Markdown snapshot is preserved under
  `archive/imported-documents/cloud-gpt-a-to-z-2026-11-05/`; its review is
  `cloud-gpt-a-to-z-review.md`. It is reference material, not active policy.
- The separate independent calibration audit's selected report, JSON and plots
  are at `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`.
  Its source ZIP and scripts remain in `C:\Users\Mohamed\Downloads\GPT_TMP\`;
  the four raw CSVs were hash-checked against repository captures.
- The user reports the aircraft is connected and is calibrating sensors. This
  does not establish that the flight-control PCB, receiver, ESC protocol, motor
  order or motor outputs have been verified.
- User-reported TOFSense-F2 Mini mounting is central underside on the F450,
  beam along body −Z; preliminary sensor-to-COM vector is `(0,0,-0.026658) m`.
  See `docs/lidar-geometry-and-altitude-fusion-2026-11-06.md`. Physical dimensions,
  sensor identity, interface, routing, and timestamps remain unverified. Keep the
  calibration GUI at 12 steps with no LiDAR stage.
- Offline `tools/altitude_fusion.py` now applies the configurable preliminary
  lever arm, manufacturer range bounds, required logged LiDAR age, and tilt/beam
  gate. Its research KF separates local z, terrain height and LiDAR clearance;
  it needs a confirmed ground pressure reference to label local altitude. It
  remains a replay prototype with assumed scalar noise, not a driver or FC
  estimator. Targeted synthetic tests and a terrain-separated example plot exist.
- Main board design evidence identifies HMC5883L footprint, while recent sensor
  capture is labeled QMC5883P. Reconcile the actual fitted device and SDA/SCL net
  route before hardware conclusions.

## Safety rules

- Physical motor arming/output remains disabled; no auto-arm or motor tests.
- Preserve original captures, simulation/evidence logs, board designs, ROS/Gazebo
  source, RViz/URDF, and linked worktree changes.
- Keep the Pico firmware runtime in one C file each: `flight_controller.c` and
  `handler_controller.c`.
- Xbox PC-to-CH343 test bridges are not validated flight receivers. Do not enable
  arming with a D-pad mapping; motor policy requires a verified adapter and stop
  path.

## Cleanup state

- Canonical plan: `docs/project-plan.md`; `C:\Users\Mohamed\Desktop\Work\PLAN.md`
  is a short entry point to that plan.
- Workspace moves and the Downloads audit:
  `docs/workspace-cleanup-2026-11-07.md`.
- Three clean Desktop worktrees were removed locally after confirming their
  GitHub branches. Dirty Stepwise, Codex, and Kilo workspaces remain untouched.
- The latest filesystem scan found uncommitted ESP32 flight-controller files in
  the canonical checkout. Preserve them and exclude them from cleanup commits.
- GPT_TMP's 29 `.pio` caches remain because recursive deletion was blocked; three
  byte-identical ZIP duplicates were removed after SHA verification.
- Canonical shared memory: `docs/thoughts/thought-database.md`.
- Cleanup decisions and archive/delete records:
  `archive/project-cleanup-manifest.md`.
- `tmp/` is tracked project content despite its name. Do not delete it wholesale.
- `build/` contains both useful scripts/source/evidence and generated caches;
  classify individually. Preserve custom ROS/Gazebo workspaces and local configs.
- The dated `build/` inventory is `docs/build-directory-inventory-2026-11-05.md`.
  It records 24,580 files (~1,311.9 MiB), including the 19-file historical
  `build/stepwise-seed/` project snapshot. Relocation was rejected before execution
  by the command safety policy; leave it in place and do not retry via another
  command.
- Keep the cloud GPT v2 ZIP candidate as bench-only and unvalidated. Its own
  manifest says host tests only; controller path is a Windows PC bridge; motors
  are compile-time locked off.

## Calibration data gate

The canonical ESP32 bench chronology and failed-trial record is
[`VALIDATION_LOG.md`](../../hardware/bench_calibration/esp32s3_qmc5883p_100hz/VALIDATION_LOG.md).
Read it before making calibration claims or changing bench procedures.

The previous handoff mentioned a LiDAR-free guided session in progress; no later
completion confirmation is recorded here. Do not select a current capture for
analysis until the user confirms completion and the session manifest/CSV show
all 12 phases reached target without skips. Preserve partial segments and
skipped-stage manifests. The user later selected the three latest fully
completed Nov 6 sessions for the raw-data handoff at
`C:\Users\Mohamed\Desktop\Work\IntelQCopter_Calibration_Handoff_Latest3_2026-11-06.zip`.
The November 4 guided session
`captures/calibration_package_20261104/iq_500hz_20261104_100907.csv` has evidence
for all 12 target phases, no skipped stages, and a matching result JSON. Raw
review confirms 38,652 fresh MAG samples at about 195 Hz during active phases;
`mag_ok` alone includes cached, non-fresh rows. It is the earlier audited
baseline, not the only completed session. Keep its candidate fit provisional:
the saved MAG fit is diagonal after a degenerate ellipsoid fit. The independent
audit reports 3.794% and 3.456% MAG magnitude RMS on separate 02:57 and 03:17
sessions (raw hashes matched locally; the analysis was not rerun in this cleanup
pass). PCB/axis and
physical heading validation remain pending. See `cloud-gpt-a-to-z-review.md`
and the imported audit report for details.

## Post-calibration DSH/PID handoff

- The Pico `flight_controller.c` snapshot still marks PID gains as placeholders,
  exposes only a software mixer, and forces `out.armed = false`; it has no
  hardware ESC output path. Its MPU configuration macros select ±500 °/s,
  ±4 g and 41 Hz, while the requested candidate is ±500 °/s, ±8 g and 188/184 Hz.
- `sensor_sample_t` carries validity booleans but no acquisition timestamps or
  sample ages. PID review must retain the latest sample each 500 Hz iteration
  without pretending reused values are fresh; add explicit age/freshness
  handling in the owner firmware before using a sample in control.
- Static review found `flight_control_step()` reads `cmd->buttons` before the
  later null/validity check, and the barometer zero button is level-triggered
  while `out.armed` is always false. Validate the command first and use a
  deliberate edge/ground-state rule before estimator zeroing. Keep the physical
  output lock unchanged.
- Do not invent numeric flight gains from calibration alone. First analyze the
  completed capture; reconcile axes, scales and units; then use motorless
  replay/SITL with measured timing and a clearly labeled plant. No receiver,
  ESC, motor-order or thrust characterization is validated in this checkout.

## Rules for the next AI

1. Read current status/diffs and this thought base before editing.
2. Append database revisions; do not rewrite measurements or historical context.
3. For cleanup, log each move/delete in the manifest. Do not delete solely based
   on a filename or a directory named `tmp`, `build`, or `old`.
4. Keep readability changes separate from behavior changes; run a diff check to
   prove formatting-only edits preserve logic.
5. Commit/push scoped milestones only; report push status accurately.
