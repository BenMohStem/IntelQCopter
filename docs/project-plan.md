# QCopter project organization and engineering plan

Updated: 2026-11-07 (Africa/Lagos)

Entry point: `C:\Users\Mohamed\Desktop\Work\PLAN.md` links back to this canonical plan.

## Guardrails

- Preserve history and all hardware/simulation evidence. Archive superseded work
  with provenance; delete only confirmed generated, reproducible clutter.
- Do not edit source behavior as part of cleanup. Separate and review every
  whitespace/comment-only readability diff.
- Pico runtime stays in `firmware/flight_controller/flight_controller.c` and
  `firmware/handler_controller/handler_controller.c`; ROS 2 keeps normal layout.
- Motor arming remains disabled. Xbox PC-to-USB tests are not flight receivers.
- The user reports the drone connected on 2026-11-06. PCB repair, fitted sensor
  identity, and wiring remain unverified in this work session; no COM port,
  upload, or physical validation was performed for the plan update.
- Keep local SDKs/environments and unrelated personal files out of GitHub.

## ESP32-S3 engineering scope added 2026-11-06

The user supplied a short-horizon ESP32-S3 flight-controller specification. The
isolated [competition-readiness plan](esp32-competition-readiness-plan-2026-11-06.md)
turns it into sensor, calibration, GUI, estimation, Xbox, control and validation
gates. Its performance values are targets until measured. Preserve the stable
fallback, the current safety lock, and the distinction between calibration/bench
work and flight validation. The detailed selected bench profile and evidence
ledger are in [the ESP32-S3 readiness plan](esp32-competition-readiness-plan-2026-11-06.md).
The aircraft is reported connected, but no new hardware data were collected.

### Current desktop and calibration status — 2026-11-07

The guided GUI currently has 12 IMU, magnetometer and BMP phases. LiDAR stages
13–16, geometry setup and LiDAR capture are deferred at the user's request;
retain the geometry requirements in
[the LiDAR note](lidar-geometry-and-altitude-fusion-2026-11-06.md) for the later
hardware step. The GUI, packet decoder, session writer, plots and Windows
Tcl/Tk preparation are consolidated in
`hardware/bench_calibration/esp32s3_qmc5883p_100hz/tools/live_calibration_gui.py`.
The PowerShell launcher calls this file directly; the old bootstrap script has
been removed. The GUI's Sensor Latency tab must only be treated as measured
when the firmware actually supplies sensor-age/service timing fields.

No new COM-port or physical hardware check was performed during this cleanup.
The PCB repair, fitted sensor identity and wiring remain unverified in this
work session. The supplied relative competition countdown is stale; the
competition outcome and exact date are not recorded here.

The sensor plan targets a fixed 400 kHz I²C bus for MPU6050, QMC5883P and
BMP280; a nonblocking 500 Hz loop that always uses the latest available sensor
snapshot; MPU 188 Hz first, then a measured 256 Hz trial, with 98 Hz only if 188
fails; ±500 °/s, ±8 g and FIFO; and the fastest verified BMP conversion with
noise/delay measured separately. These remain targets. The 2026-11-06 DSH
working-tree inspection recorded an earlier 42/44 Hz, ±250 °/s, ±2 g,
FIFO-disabled MPU setup, QMC clock switching to 100 kHz, and a 52-byte v3
stream. Treat that as a dated snapshot, not current source status; review the
active candidate and its validation records before relying on it. See the
detailed [readiness plan](esp32-competition-readiness-plan-2026-11-06.md).

The user also requested a cable-free calibration session for easier 3D
magnetometer tumbling. The desktop launcher and GUI contain a Wi-Fi/UDP option,
but end-to-end capture, v4 LiDAR forwarding, and magnetic interference still
require verification. Codex owns desktop GUI and analysis work; DSH owns ESP32
firmware and sensor telemetry. Motor outputs remain disabled.

The Nov 6 raw-data handoff contains the three latest sessions the user identified
as fully completed (start times 20:22, 20:50 and 20:59 Africa/Lagos). It preserves
about 187 MB of original CSV data in a roughly 32 MB ZIP at
`C:\Users\Mohamed\Desktop\Work\IntelQCopter_Calibration_Handoff_Latest3_2026-11-06.zip`.
This package is the primary input for any further fit; it does not itself validate
calibration constants or PID gains. The separate Nov 4 session remains the
previously audited baseline: 98,868 rows, exact targets for 12 phases, empty skip
manifest and matching result JSON. Its settings were the older ±250 °/s, ±2 g
and low-DLPF profile. Other Nov 6 captures with unresolved launch or phase-attempt
mapping remain excluded unless their provenance is resolved.

Keep continuous QMC, advanced estimator/filter research and RP2350 migration in
separate follow-up work. Do not use the old relative competition countdown as a
current schedule; the outcome and exact event date have not been confirmed.

## Cleanup phases

### A. Knowledge and handoff — complete

- Consolidated the shared thought base under `docs/thoughts/` with an index,
  current coordination summary, and append-only canonical database.
- Preserved the historical root thought database under
  `archive/thought-bases/2026-11-05/`; old path is a compatibility pointer.
- Added current plan, calibration admission evidence, cloud ZIP limits, and DSH
  work review notes to the canonical knowledge base.
- Imported and reviewed the Cloud GPT A-to-Z documentation snapshot. Its source
  is archived verbatim with its hash manifest; supported capture facts were
  checked against the raw CSV and unsupported/contradictory claims are labeled
  in `docs/thoughts/cloud-gpt-a-to-z-review.md`.

### B. Inventory and safe cleanup — active (workspace consolidation updated 2026-11-07)

1. Maintain `archive/project-cleanup-manifest.md` for every disposition.
2. Preserve `tmp/` because it is tracked source, lessons, and evidence.
3. Preserve `build/` contents until classified; it includes helper scripts,
   reports, local ROS/Gazebo workspaces and generated caches.
4. Remove only confirmed standalone caches with no active process using them;
   never remove custom ROS/Gazebo, RViz/URDF or original evidence.
5. Move old source snapshots and generated package archives to clearly named
   archive paths only after recording hash, provenance, and whether they stay
   local-only.

Inventory update (2026-11-05): the ignored `build/` tree has 24,580 files and is
about 1,311.9 MiB. It mixes dependency environments and caches with local ROS
workspaces, reports, datasets, scripts, CAD trials, and source/evidence snapshots.
See the [local build inventory](build-directory-inventory-2026-11-05.md).
The 19-file `build/stepwise-seed/` snapshot is a complete small bench-project
copy, not a disposable build product. A copy-verify-relocate operation into
`archive/source-snapshots/` was rejected by the command safety policy before
execution; the source remains untouched in `build/`. Do not retry through another
command. Continue with documentation and non-destructive classification.

The Desktop workspace consolidation and Downloads\GPT_TMP audit are recorded in
[the workspace cleanup report](workspace-cleanup-2026-11-07.md). Three clean
duplicate worktrees were removed after verifying their remote branches; dirty
AI workspaces were preserved. The canonical checkout's DSH changes and the
calibration analysis outputs remain uncommitted and must not be included in an
unrelated cleanup commit. `GPT_TMP` still has 29 generated `.pio` trees; the
recursive cache removal was blocked before any of those trees changed. Three
byte-identical ZIP copies were removed after hash verification. Keep the mixed
repository `build/` tree classified separately and do not bulk-delete it.

### C. Readability — pending

- Review project source formatting subsystem by subsystem.
- Apply only consistent indentation, line wrapping, and explanatory comments;
  do not alter registers, constants, control flow, units, pins, safety gates, or
  algorithms in this phase.
- For each file, compare normalized tokens or use a formatting-only diff review;
  keep behavior changes separate and review the archived DSH candidate before
  adopting its sensor register/configuration edits.

### D. Calibration analysis — candidate audit imported; independent rerun pending

- The user's three-session Nov 6 handoff is the current raw-data input for
  analysis. It contains the latest three fully completed sessions selected by
  the user. The 20:59 capture preserves redo attempts; use the newest completed
  attempt for each phase when fitting. No fit or PID gain is validated by the
  raw-data package itself.
- The Nov 4 10:09 session remains an earlier independent audit baseline. It has
  12 phase attempts; target accepted counts (30,000 stationary, six × 5,000
  accelerometer, three × 2,000 magnetic turns, 8,000 MAG tumble, 1,000 BMP);
  an empty skipped-stage list; matching calibration JSON; 38,652 fresh MAG rows
  across the capture; and about 195 Hz during active phase windows.
- The independent audit report, candidate JSON/header, corrected CSV derivative,
  model comparison and plots are organized beside the captures at
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`.
  It reports gyro bias/noise, six-face accel correction, full SPD MAG fit,
  bootstrap, session holdouts, barometer variance and limitations.
- Selected repeatable MAG baseline: QMC5883P at `0x2C` (ID `0x80`), normal
  low-OSR 200 Hz (`CTRL1=0x3D`, `CTRL2=0x08`, `XYZ_SIGN=0x06`), 100 kHz QMC
  transactions with STOP+START reads, and freshness gated by status DRDY plus a
  successful six-byte XYZ read. The capture source switches to 400 kHz for
  MPU/BMP and back to 100 kHz for QMC; global 400 kHz remains unproven. Isolated
  tests measured about 188–189 fresh/s;
  the complete November 4 capture recorded about 195 fresh flags/s in active
  phases, but integrated 400 kHz results conflict. This remains historical
  evidence and a diagnostic fallback only. The user's current target is a fixed
  400 kHz shared bus; see the readiness plan's new revision and test gates.
- Reproduce or independently review the audit computations from the verified
  raw CSVs before approving calibration constants. Record methods, versions,
  axes, units and limitations; check the fresh-sample gate and matrix direction.
- Keep fits provisional. Existing MAG fit fell back to diagonal after full
  ellipsoid degeneracy; PCB identity and physical axis mapping need resolution.
- A-to-Z reports independent-session MAG errors. Its audit ZIP was found in
  `Downloads\GPT_TMP`; four raw CSVs hash-match the repository captures and
  selected report/results/plots are imported. The audit scripts were not rerun
  in this pass. Keep all outputs provisional.

### E. Xbox/FC review — pending

- Compare the Xbox worktree and cloud GPT v2 bench package. Keep virtual PC
  bridge separate from an actual flight receiver.
- For the ESP32-S3 competition candidate, use 10% armed idle / 35% final
  per-motor normal cap / 50% final per-motor boost cap. Keep physical motor
  output disabled until its adapter and safety gates are verified. No D-pad
  arming, flash, or motor test.

### F. Milestone validation and publication

- Make scoped changes, inspect diffs, run requested offline checks, record engine
  and assumptions, commit and push after each milestone when possible.
- Never call a compile, simulation, or PC bridge a physical validation.

## Current hardware identification gate

The user reports the drone connected. Available PCB evidence shows an
HMC5883L compass footprint while calibration captures identify a QMC5883P.
Confirm the fitted part and SDA/SCL routing from the repaired board before using
the configuration for flight. No hardware test was run for this plan update.
