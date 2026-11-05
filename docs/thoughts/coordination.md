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
- The Cloud GPT A-to-Z Markdown snapshot is preserved under
  `archive/imported-documents/cloud-gpt-a-to-z-2026-11-05/`; its review is
  `cloud-gpt-a-to-z-review.md`. It is reference material, not active policy.
- The separate independent calibration audit's selected report, JSON and plots
  are at `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`.
  Its source ZIP and scripts remain in `C:\Users\Mohamed\Downloads\GPT_TMP\`;
  the four raw CSVs were hash-checked against repository captures.
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

The November 4 guided session
`captures/calibration_package_20261104/iq_500hz_20261104_100907.csv` has evidence
for all 12 target phases, no skipped stages, and a matching result JSON. Raw
review confirms 38,652 fresh MAG samples at about 195 Hz during active phases;
`mag_ok` alone includes cached, non-fresh rows. It is the only identified fully
completed session. Keep candidate fits provisional: the saved MAG fit is
diagonal after a degenerate ellipsoid fit. The independent audit reports 3.794%
and 3.456% MAG magnitude RMS on separate 02:57 and 03:17 sessions (raw hashes
matched locally; the analysis was not rerun in this cleanup pass). PCB/axis and
physical heading validation remain pending. See `cloud-gpt-a-to-z-review.md`
and the imported audit report for details.

## Rules for the next AI

1. Read current status/diffs and this thought base before editing.
2. Append database revisions; do not rewrite measurements or historical context.
3. For cleanup, log each move/delete in the manifest. Do not delete solely based
   on a filename or a directory named `tmp`, `build`, or `old`.
4. Keep readability changes separate from behavior changes; run a diff check to
   prove formatting-only edits preserve logic.
5. Commit/push scoped milestones only; report push status accurately.
