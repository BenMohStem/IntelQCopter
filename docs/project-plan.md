# QCopter project organization and engineering plan

Updated: 2026-11-06 (Africa/Lagos)

Convenience copy: `C:\Users\Mohamed\Desktop\Work\PLAN.md`.

## Guardrails

- Preserve history and all hardware/simulation evidence. Archive superseded work
  with provenance; delete only confirmed generated, reproducible clutter.
- Do not edit source behavior as part of cleanup. Separate and review every
  whitespace/comment-only readability diff.
- Pico runtime stays in `firmware/flight_controller/flight_controller.c` and
  `firmware/handler_controller/handler_controller.c`; ROS 2 keeps normal layout.
- Motor arming remains disabled. Xbox PC-to-USB tests are not flight receivers.
- Drone is disconnected for PCB repair. No COM3, upload, or physical validation.
- Keep local SDKs/environments and unrelated personal files out of GitHub.

## ESP32-S3 engineering scope added 2026-11-06

The user supplied a short-horizon ESP32-S3 flight-controller specification. The
isolated [competition-readiness plan](esp32-competition-readiness-plan-2026-11-06.md)
turns it into sensor, calibration, GUI, estimation, Xbox, control and validation
gates. Its performance values are targets until measured. Preserve the stable
fallback, the current safety lock, and the distinction between calibration/bench
work and flight validation. The aircraft remains disconnected for PCB repair.

The new specification requests an optional LiDAR calibration path and a revised
Xbox mapping; these are planning updates only. The current 12-step GUI remains
available, LiDAR stays optional, and no behavior is adopted in firmware until the
source audit and required hardware/transport evidence pass.

The competition window is described as approximately two days away in the
supplied text, but its date is unconfirmed. Keep the release scope conservative;
defer continuous QMC, advanced estimator/filter research and RP2350 migration to
separate post-window work.

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

### B. Inventory and safe cleanup — active

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

### C. Readability — pending

- Review project source formatting subsystem by subsystem.
- Apply only consistent indentation, line wrapping, and explanatory comments;
  do not alter registers, constants, control flow, units, pins, safety gates, or
  algorithms in this phase.
- For each file, compare normalized tokens or use a formatting-only diff review;
  keep behavior changes separate and review the archived DSH candidate before
  adopting its sensor register/configuration edits.

### D. Calibration analysis — candidate audit imported; independent rerun pending

- Current candidate complete session is the November 4 10:09 capture. Raw
  evidence: 12 phase attempts; target accepted counts (30,000 stationary, six ×
  5,000 accelerometer, three × 2,000 magnetic turns, 8,000 MAG tumble, 1,000
  BMP); empty skipped-stage list; matching calibration JSON; 38,652 fresh MAG
  rows across the capture and ~195 Hz within active phase windows.
- The independent audit report, candidate JSON/header, corrected CSV derivative,
  model comparison and plots are organized beside the captures at
  `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`.
  It reports gyro bias/noise, six-face accel correction, full SPD MAG fit,
  bootstrap, session holdouts, barometer variance and limitations.
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
- Maintain current 10% armed idle / 45% normal cap / 65% boost cap policy and
  physical arming lock. No D-pad arming, flash, or motor test.

### F. Milestone validation and publication

- Make scoped changes, inspect diffs, run requested offline checks, record engine
  and assumptions, commit and push after each milestone when possible.
- Never call a compile, simulation, or PC bridge a physical validation.

## Current PCB blocker

The drone is disconnected pending PCB repair. Available PCB evidence shows an
HMC5883L compass footprint; calibration evidence labels the sensor QMC5883P.
Resolve fitted part and SDA/SCL routing from board files/photos after repair,
before any hardware test or flight parameter deployment.
