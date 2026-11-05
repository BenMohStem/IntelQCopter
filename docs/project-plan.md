# QCopter project organization and engineering plan

Updated: 2026-11-05 (Africa/Lagos)

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

## Cleanup phases

### A. Knowledge and handoff — complete

- Consolidated the shared thought base under `docs/thoughts/` with an index,
  current coordination summary, and append-only canonical database.
- Preserved the historical root thought database under
  `archive/thought-bases/2026-11-05/`; old path is a compatibility pointer.
- Added current plan, calibration admission evidence, cloud ZIP limits, and DSH
  work review notes to the canonical knowledge base.

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

### C. Readability — pending

- Review project source formatting subsystem by subsystem.
- Apply only consistent indentation, line wrapping, and explanatory comments;
  do not alter registers, constants, control flow, units, pins, safety gates, or
  algorithms in this phase.
- For each file, compare normalized tokens or use a formatting-only diff review;
  keep behavior changes separate and review the archived DSH candidate before
  adopting its sensor register/configuration edits.

### D. Calibration analysis — pending offline analysis

- Current candidate complete session is the November 4 10:09 capture. Evidence:
  12 phase attempts; target accepted counts (30,000 stationary, six × 5,000
  accelerometer, three × 2,000 magnetic turns, 8,000 MAG tumble, 1,000 BMP);
  empty skipped-stage list and matching calibration JSON.
- Recompute from raw CSV: offsets, full scale/misalignment, per-axis standard
  deviations/covariance, residuals, coverage, radial field statistics, held-out
  evaluation, and plots. Record methods, versions, axes, units and limitations.
- Keep fits provisional. Existing MAG fit fell back to diagonal after full
  ellipsoid degeneracy; PCB identity and physical axis mapping need resolution.

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
