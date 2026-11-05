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

## Classified and retained

| Path | Classification | Decision |
|---|---|---|
| `tmp/` | Tracked source, lessons, and evidence | Keep; the directory name does not make it disposable |
| `build/` | Mixed helper scripts, reports, ROS/Gazebo workspaces, and generated caches | Keep pending precise classification; preserve custom sources and evidence |
| `.local/` | Machine-local configuration | Keep local and exclude from publication |
| `output/pdf/` | Generated project deliverables | Keep |
| `artifacts/`, `captures/`, `measurements/`, PCB designs, simulation logs | Primary evidence | Keep, including failed trials |

## Deletions

No project source, calibration data, simulation evidence, cache, or local
environment was deleted in this pass. Cache deletion was not completed; retain
the existing build tree until its precise contents and active use are reviewed.
