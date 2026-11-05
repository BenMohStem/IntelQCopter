# Local build directory inventory

Inventory date: 2026-11-05 (Africa/Lagos)
Scope: ignored `build/` directory in the QCopter checkout. These are local files,
not part of the Git tree or a proposed GitHub upload.

## Summary

The recursive inventory counted 24,580 files, totaling approximately 1,311.9 MiB
(1.28 GiB). The directory is mixed-use: generated build output, installed Python
and formatting dependencies, local ROS workspaces, scripts, reports, datasets,
CAD trials, and source/evidence snapshots all sit together. A name such as
`build/`, `tmp`, or `cache` is not enough to decide that content is disposable.

## Largest inventoried areas

| Path | Files | Approx. size | Classification |
|---|---:|---:|---|
| `build/module-dimensions/` | 4,016 | 233.7 MiB | Mixed CAD scripts, dependencies, source inputs and review outputs; inspect by subfolder |
| `build/route-pcb/` | 5,453 | 230.9 MiB | PCB route trials, user sketch and dependencies; preserve trial evidence |
| `build/pcb-audit-python/` | 2,525 | 128.8 MiB | Local SciPy/Python environment; machine-local dependency, not publishable |
| `build/zig-cache/` | 2,788 | 125.2 MiB | Zig compiler cache; deletion was rejected by the command safety policy; leave untouched |
| `build/imu-analysis-deps/` | 4,058 | 120.5 MiB | Local analysis environment; machine-local dependency, not publishable |
| `build/ros2/` | 552 | 81.1 MiB | Local ROS workspace with `qcopter_autonomy`, `qcopter_description` and `cv_bridge`; preserve pending workspace review |
| `build/friend-arduino-check/` | 528 | 72.0 MiB | Arduino/PlatformIO check with source and build evidence; preserve pending ownership/scope review |
| `build/ros_python/` | 1,400 | 49.9 MiB | Local ROS Python packages; machine-local dependency, not publishable |
| `build/tectonic-cache/` | 499 | 45.5 MiB | Tectonic cache; deletion was rejected by the command safety policy; leave untouched |
| `build/stepwise-seed/` | 19 | small | Historical ESP32-S3 bench-project snapshot, including firmware, hardware assumptions and validation plan; not a generated cache |

Other notable groups include `controller-rp2040/`, `flight-rp2350/`, `host-tests/`,
`sitl/`, and `ros2-install/` build outputs; `build/tools/tectonic.exe`; source and
trial folders for PCB work; PDFs, render previews and QA screenshots; and datasets
for panel/perception work. Preserve custom Gazebo/ROS/RViz/URDF material and the
original evidence while reviewing each area.

## Source snapshot in build

`build/stepwise-seed/` contains 19 files and no `.git` directory. Its README says it
is an ESP32-S3 board-diagnostics project with no flight controller or motor-output
driver. It includes an unconfirmed ESP32 target, GPIO/I2C assumptions, diagnostic
firmware, a staged validation plan, and small command-line tools. It is distinct
from the separately registered `QCopter-Stepwise` worktree; most files differ from
that worktree's current HEAD. The original snapshot has not been modified.

A copy-and-verify relocation to `archive/source-snapshots/` was attempted so its
source would not remain under `build/`. The command safety policy rejected that
operation before execution. No copy or deletion occurred. Do not retry this move
through another command. Keep this snapshot intact and record the limitation in
`archive/project-cleanup-manifest.md` and `docs/project-plan.md`.

## Safe handling

- `build/` is ignored by Git. Keep local environments, SDKs, downloaded tools and
  unrelated coursework out of commits and pushes.
- Preserve original captures, failed trials, local ROS/Gazebo workspaces, CAD
  designs and source snapshots until their provenance and recovery path are clear.
- Do not bulk-delete the tree. The two compiler-cache removal requests for
  `build/zig-cache/` and `build/tectonic-cache/` were rejected; do not bypass that
  guard or retry by a different method.
- Review generated outputs and local course/report assets individually against
  the sibling `IntelQCopter-USTHB` worktree before any later disposition.
- The `docs/` subtree was checked for byte-identical Markdown, text and JSON files;
  none were found. That check does not establish that files outside `docs/` are
  duplicates.

No local build files were moved or deleted during this inventory. The root
`.gitignore` keeps `build/` excluded from GitHub.
