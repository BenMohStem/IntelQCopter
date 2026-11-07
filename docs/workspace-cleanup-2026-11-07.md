# Workspace cleanup — 2026-11-07

## Canonical project and local workspace layout

Use `C:\Users\Mohamed\Desktop\Work\QCopter` as the canonical checkout. The full
project plan is `docs/project-plan.md`; `C:\Users\Mohamed\Desktop\Work\PLAN.md`
is now a short entry point to that single canonical plan.

Three clean duplicate checkouts were removed from `Desktop\Work` after their
branches were confirmed on `origin`:

| Removed local worktree | Branch retained on GitHub | Last verified commit |
|---|---|---|
| `QCopter-codex-lidar` | `codex/lidar-geometry-latency-2026-11-06` | `70bbbff` |
| `IntelQCopter-USTHB` | `codex/usthb-university-report` | `7e77c87` |
| `QCopter-ESP32-Xbox` | `esp32-xbox` | `af29870` |

The Xbox checkout's only untracked content was a KiCad output directory. It was
preserved under the ignored local archive at
`.local/archives/workspaces/esp32-xbox-output/`, with `SHA256SUMS.txt` for all
72 files. The removed checkouts' generated `.pio` contents were discarded with
those worktrees; the canonical checkout and its build cache were not changed.

Dirty or AI-managed workspaces were left in place: `QCopter-Stepwise`, the Codex
motor-limits worktree, and all three Kilo worktrees. Their edits remain local and
were not merged or cleaned.

## Desktop organization

Twenty-eight loose files and folders from the top of `Desktop\Work` were moved
under the ignored `.local/archives/desktop-work/` tree in the canonical checkout.
This includes the extracted Nov 6 handoff stage, all four distinct handoff ZIP
variants, the separate calibration handoff folder and ZIP, old analysis/debug
scripts, Isaac scan/result JSONs and DSH prompt notes. The two project `.7z`
backups now live in `.local/archives/backups/`. These files were moved, not
deleted, and are intentionally excluded from GitHub.

The unrelated `Drone`, coursework and other project folders in `Desktop\Work`
were not moved. The `QCopter-Stepwise` workspace was preserved because it is
dirty and contains untracked evidence and code.

## Downloads\GPT_TMP audit

The folder contained 7,552 files and about 1.97 GiB before cleanup. Its three
newest Nov 7 candidate bundles were reviewed as research inputs; their package
notes do not prove a PlatformIO target build or physical behavior. The 4-file
and 5-file bundles have identical `src/main.cpp` content but differing other
files. No candidate firmware was imported.

Three ZIP files named with `(1)` were removed after SHA-256 confirmed each was
byte-identical to its retained counterpart. This freed 10,921,465 bytes. Source
trees, raw captures, logs, reports and unique archives remain in place.

The folder also contains 29 generated `.pio` trees totaling 1,274,887,985 bytes
(1,215.83 MiB). Their removal was blocked by the Windows command safety review;
none of those caches was changed. The inventory and exact paths are in the
local-only `C:\Users\Mohamed\Downloads\GPT_TMP\CLEANUP_MANIFEST_2026-11-07.md`.
`Downloads\Tmp` is a separate folder and was not touched.

## Documentation consolidation

`docs/project-plan.md` is the canonical full plan; the file in `Desktop\Work` is
only a launch point. Subsystem guides remain separate because they document
different evidence, interfaces or procedures. The byte-identical calibration
README inside the imported A-to-Z snapshot remains unchanged to preserve the
source snapshot and its provenance.

## Credential hygiene

The current tracked copy of the archived DSH candidate had a hard-coded Wi-Fi
password. That one value was replaced with a placeholder before publishing this
cleanup. An ignored local backup preserves the original bytes at
`.local/security-review/`; the Git history was not rewritten. If the access
point still uses that password, rotate it. The Nov 3 handoff ZIP added to the
local `main` branch passed a basic embedded-secret scan before publication.

## Validation limits

This was a filesystem and Git organization pass. No GUI launch, compile, build,
flash, COM-port access, sensor test or flight test was performed. Removing local
worktrees does not remove their Git branches; each can be checked out again.
