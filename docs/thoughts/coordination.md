# QCopter AI coordination

Updated: 2026-11-05 (Africa/Lagos)

Read in this order: `AGENTS.md`, this summary, `thought-database.md`, then the
relevant subsystem README and `VALIDATION_LOG.md`.

## Current state

- Branch: `codex/qmc-200hz-continuous-diagnosis`.
- DeepSeek stopped. Its uncommitted calibration `src/main.cpp` candidate is
  preserved for review at
  `archive/review-candidates/2026-11-05/qmc5883p-main-dsh-working-tree.cpp`;
  the working-tree source was restored to the current commit during cleanup.
  Review the archived candidate before adopting its register/configuration edits.
- Drone is disconnected for PCB repair. Do not use COM3 or claim hardware
  validation until the user says the board is repaired and connected.
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

- Canonical plan: `docs/project-plan.md`; convenience copy:
  `C:\Users\Mohamed\Desktop\Work\PLAN.md`.
- Canonical shared memory: `docs/thoughts/thought-database.md`.
- Cleanup decisions and archive/delete records:
  `archive/project-cleanup-manifest.md`.
- `tmp/` is tracked project content despite its name. Do not delete it wholesale.
- `build/` contains both useful scripts/source/evidence and generated caches;
  classify individually. Preserve custom ROS/Gazebo workspaces and local configs.
- Keep the cloud GPT v2 ZIP candidate as bench-only and unvalidated. Its own
  manifest says host tests only; controller path is a Windows PC bridge; motors
  are compile-time locked off.

## Calibration data gate

The November 4 guided session `captures/calibration_package_20261104/iq_500hz_20261104_100907.csv`
has evidence for all 12 target phases, no skipped stages, and a matching result
JSON. It is currently the only identified fully completed session. Recompute
from raw rows, inspect per-row validity/freshness and fit coverage, and preserve
provisional status because the MAG fit is diagonal after a degenerate ellipsoid
fit and PCB/axis validation is pending.

## Rules for the next AI

1. Read current status/diffs and this thought base before editing.
2. Append database revisions; do not rewrite measurements or historical context.
3. For cleanup, log each move/delete in the manifest. Do not delete solely based
   on a filename or a directory named `tmp`, `build`, or `old`.
4. Keep readability changes separate from behavior changes; run a diff check to
   prove formatting-only edits preserve logic.
5. Commit/push scoped milestones only; report push status accurately.
