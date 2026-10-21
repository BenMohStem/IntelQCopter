# Historical source and setup procedures

`isaac/` contains the retired Isaac integration from main at commit 3803437.
`tools/` contains one-off installation/repair, repository-creation and historical
report generators from the same commit. Their original bytes are preserved.
They retain their original path assumptions and are not active launch commands.
Use `git show 3803437:<original-path>` or a separate checkout at that commit when
reproducing historical behavior.

Original logs, failed trials and recordings remain in `artifacts/`. They are
evidence, not disposable cache. Current commands are listed by `qcopter.ps1 help`.
The root `project.json` and docs/architecture.md describe the active layout.
