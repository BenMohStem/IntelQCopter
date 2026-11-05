# Archive index

Use `project-cleanup-manifest.md` for dated move, retention, and deletion
records. Keep archived documents and code as historical evidence; do not treat
them as active project instructions or firmware settings without review.

## Imported reference documents

- `imported-documents/cloud-gpt-a-to-z-2026-11-05/` contains the owner's A-to-Z
  Markdown snapshot and a readme with its source ZIP hash and verified manifest.
- `thought-bases/` contains the byte-preserved historical thought database.
- `review-candidates/` contains unreviewed DSH working-tree source preserved for
  comparison; it is not adopted firmware.
- `source-snapshots/` contains distinct prior source versions retained with
  provenance and checksums.
- `local-packages/` contains generated or oversized packages that stay excluded
  from Git.

## Historical source and setup procedures

`isaac/` contains the retired Isaac integration from main at commit 3803437.
`tools/` contains one-off installation/repair, repository-creation and historical
report generators from the same commit. Their original bytes are preserved.
They retain their original path assumptions and are not active launch commands.
Use `git show 3803437:<original-path>` or a separate checkout at that commit when
reproducing historical behavior.

Original logs, failed trials and recordings remain in `artifacts/`. They are
evidence, not disposable cache. Current commands are listed by `qcopter.ps1 help`.
The root `project.json` and docs/architecture.md describe the active layout.
