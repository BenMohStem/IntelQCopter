# Project cleanup manifest

Started: 2026-11-05 (Africa/Lagos)

This file records cleanup dispositions and provenance. Preserve original
calibration captures, simulation logs, failed trials, and hardware evidence.

## Organized and archived

| Date | Original | Action | Destination / rationale |
|---|---|---|---|
| 2026-11-05 | Root thought database | Moved byte-for-byte | `archive/thought-bases/2026-11-05/thought-database-2026-11-03-original.md` |
| 2026-11-05 | Duplicate DSH thought database `docs/thoughts/thought-database-2026-11-03-rev4.md` | Renamed as canonical after matching hashes | `docs/thoughts/thought-database.md` |
| 2026-11-05 | Historical thought-base path | Replaced with link pointer | `docs/thought-database-2026-11-03.md`; original retained above |
| 2026-11-05 | Thought index and current-state handoff | Renamed and organized | `docs/thoughts/README.md`, `docs/thoughts/coordination.md` |
| 2026-11-05 | Project plan | Added repository copy and requested convenience copy | `docs/project-plan.md` and `C:\Users\Mohamed\Desktop\Work\PLAN.md` |
| 2026-11-05 | `firmware/flight_controller/flight_controller.c.tmp` | Archived distinct 140,826-byte source snapshot; not merged | `archive/source-snapshots/2026-11-04/flight_controller-before-ybaro-lt-2026-11-04.c`; SHA-256 `A82FF214F524941E6CDCE734F67F64F4E3FEDEF1D075479F9784B0CD30DCE879` |
| 2026-11-05 | Uncommitted calibration `src/main.cpp` working-tree candidate | Preserved for review, not adopted into cleanup commit | `archive/review-candidates/2026-11-05/qmc5883p-main-dsh-working-tree.cpp`; SHA-256 `4F9CDFDC69F9532E4B1AE4A1C7245691CE348425D6367DD339B89EBC7C399C0A`. Contains unreviewed register/configuration changes; restore baseline remains in the working tree. |
| 2026-11-05 | `hardware/bench_calibration/esp32s3_qmc5883p_100hz.zip` | Moved to local-only archive; ZIP integrity checked, 780 entries, includes `.pio` build outputs | `archive/local-packages/esp32s3_qmc5883p_100hz-with-pio-build.zip`; SHA-256 `11A4701625904A8112451D4F71C230E82AFB2A19FA25CB385204A290E66A62FB`; excluded from Git |
| 2026-11-05 | Owner-supplied `IntelQCopter_A_to_Z_Markdown_Documentation_2026-11-05.zip` (outside repository) | Extracted all 91 Markdown files unchanged after verifying all 90 listed file hashes and sizes | `archive/imported-documents/cloud-gpt-a-to-z-2026-11-05/`; source ZIP SHA-256 `839C1B5605BE5325C58E0F99F52E8FEA4DFAB122A7BEB4D26452F7D380D99E6A`; the original ZIP remains at its supplied path |
| 2026-11-05 | `Downloads/GPT_TMP/IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip` | Verified all 40 internal checksums and raw CSV identities; imported selected report/results/plots without duplicate raw captures | `hardware/bench_calibration/esp32s3_qmc5883p_100hz/captures/independent_audit_20261104/`; ZIP SHA-256 `7C55BA598286AA67B2BFA5BDB8E70560E952B2FC521470F40E59D2749BF6C4BC`; full source ZIP remains local in `Downloads/GPT_TMP` |
| 2026-11-07 | `hardware/bench_calibration/esp32s3_qmc5883p_100hz/LIDAR_GUI_CHANGES.txt` | Moved unchanged out of the active package root; its default-LiDAR-stage instructions are superseded | `archive/calibration/esp32s3_qmc5883p_100hz/2026-11-03/`; SHA-256 `48E972C13C53D62018840F8B6796B4961885CC0F0D358F497B2CAEAEDDB967B7` |
| 2026-11-07 | `hardware/bench_calibration/esp32s3_qmc5883p_100hz/MAG_REPAIR_NOTES.txt` | Moved unchanged out of the active package root; proposed changes are not validation evidence | `archive/calibration/esp32s3_qmc5883p_100hz/2026-11-03/`; SHA-256 `3942DE0D8A943738F7C3AB186579C322CF1F0CF2608AF111CA1E4D9B7BBC0248` |

## Reviewed but not moved

| Date | Path | Findings | Disposition |
|---|---|---|---|
| 2026-11-05 | `build/stepwise-seed/` | 19-file ESP32-S3 bench-project snapshot (plan, validation tools, hardware assumptions, board-diagnostic firmware); no `.git` directory; not a generated cache. A separate `QCopter-Stepwise` worktree exists. | Copy-and-verify relocation to `archive/source-snapshots/` was rejected by the command safety policy before execution. No copy or deletion occurred. Keep the source intact; do not retry via another command. |

## Local build inventory

`docs/build-directory-inventory-2026-11-05.md` records 24,580 files (~1,311.9 MiB)
under ignored `build/`. It includes source/evidence, reports, datasets, local ROS
workspaces, installed dependencies, generated outputs, and caches. It is not safe
to bulk-delete or publish. The command safety policy previously rejected removal
of `build/zig-cache` and `build/tectonic-cache`; leave both untouched and do not
retry those removals through another route.

## Classified and retained

| Path | Classification | Decision |
|---|---|---|
| `tmp/` | Tracked source, lessons, and evidence | Keep; the directory name does not make it disposable |
| `build/` | Mixed helper scripts, reports, ROS/Gazebo workspaces, and generated caches | Keep pending precise classification; preserve custom sources and evidence |
| `.local/` | Machine-local configuration | Keep local and exclude from publication |
| `output/pdf/` | Generated project deliverables | Keep |
| `artifacts/`, `captures/`, `measurements/`, PCB designs, simulation logs | Primary evidence | Keep, including failed trials |

## Deletions

This section records the earlier cleanup pass. See the dated update below for
the 2026-11-07 workspace consolidation and verified duplicate-download cleanup.

## 2026-11-07 workspace consolidation and local archive update

| Date | Path | Action | Result |
|---|---|---|---|
| 2026-11-07 | `Desktop\Work\QCopter-codex-lidar` | Removed clean local linked worktree after verifying remote branch and commit | Branch `codex/lidar-geometry-latency-2026-11-06` remains on `origin` at `70bbbff`; local generated files/build cache removed with checkout |
| 2026-11-07 | `Desktop\Work\IntelQCopter-USTHB` | Removed clean local linked worktree after verifying remote branch and commit | Branch `codex/usthb-university-report` remains on `origin` at `7e77c87` |
| 2026-11-07 | `Desktop\Work\QCopter-ESP32-Xbox` | Removed clean local linked worktree after archiving its only untracked output | Branch `esp32-xbox` remains on `origin` at `af29870`; generated local build files removed with checkout |
| 2026-11-07 | Xbox worktree KiCad output (72 files, 2,262,126 bytes) | Moved intact to ignored local storage; hashes recorded | `.local/archives/workspaces/esp32-xbox-output/`; see `SHA256SUMS.txt` |
| 2026-11-07 | 28 loose files/folders from `Desktop\Work` | Moved, not deleted | `.local/archives/desktop-work/`; retained handoff versions, calibration handoff, prompts, old helper scripts, and Isaac JSON reports; total 991,755,430 bytes |
| 2026-11-07 | Two project `.7z` backups in the checkout | Moved, not deleted | `.local/archives/backups/`; excluded from Git |
| 2026-11-07 | Three `(1).zip` copies in `Downloads\GPT_TMP` | Deleted after SHA-256 equality with retained ZIP was verified | Duplicate hashes: `090020AC3B64E87F98C0D1013BFA9DC383483E7DEA313F99EF50B958E6BC1036`, `10E551E78570C13B0F8E87117816C28ECB462658F57D12623C723800C46EC3F5`, `ACE33912AC963A0FC689D2788CC8067B2F99870D8D8F17ED4D39456359D30089`; 10,921,465 bytes freed |
| 2026-11-07 | `Downloads\GPT_TMP\**\.pio` | Inventoried but not deleted | 29 cache directories, 1,274,887,985 bytes; recursive deletion was blocked by Windows command safety review before any cache changed. Details: `Downloads\GPT_TMP\CLEANUP_MANIFEST_2026-11-07.md` |
| 2026-11-07 | `archive/review-candidates/2026-11-05/qmc5883p-main-dsh-working-tree.cpp` | Replaced embedded Wi-Fi password in current copy with a placeholder before publication | Original preserved locally at ignored `.local/security-review/`; history not rewritten. Rotate the AP password if still active. |

Dirty Stepwise, Codex motor-limits, and Kilo workspaces were not altered. The
new uncommitted ESP32 flight-controller and calibration-analysis files found in
the canonical checkout were also preserved and excluded from this cleanup.
No GUI, test, compile, flash, COM-port, sensor, or flight validation was run.
