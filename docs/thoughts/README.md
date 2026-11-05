# IntelQCopter Thought Base Index

This directory is the single organized home for all thought-base / shared-working-memory
files used by concurrent AI models working on the IntelQCopter project.

**Active shared database:** [`thought-database.md`](thought-database.md). This is
the single canonical source. The old path `docs/thought-database-2026-11-03.md`
is a compatibility pointer; its original contents are preserved in
`archive/thought-bases/2026-11-05/`.

---

## Files

| File | Date | Owner | Scope | Status |
|------|------|-------|-------|--------|
| `thought-database.md` | 2026-11-05 | shared | Full project history, current status, evidence references, and handoffs | Active — append revisions |
| `coordination.md` | 2026-11-05 | shared | One-page current-state summary for any model joining the work | Active |
| `cloud-gpt-a-to-z-review.md` | 2026-11-05 | shared | Evidence review and discrepancy list for the imported A-to-Z reference package | Active reference |
| `../build-directory-inventory-2026-11-05.md` | 2026-11-05 | shared | Local ignored build-tree inventory, safe classifications, and blocked cleanup record | Active reference |

---

## Rules for any model joining this work

1. **Read first:** `coordination.md`, then `thought-database.md`,
   then `QCopter/AGENTS.md`, then `QCopter/docs/VALIDATION_LOG.md` (live lab notebook).
2. **Check git state:** `git log --oneline -10` and `git status` before acting.
3. **Do not overwrite history:** Append a dated revision at the bottom of the thought
   database rather than editing earlier revisions in place.
4. **Do not both drive COM3.** Claim the board explicitly before a run; end the claim
   when done.
5. **Preserve all evidence:** never delete or overwrite captures, artifacts, archive,
   or GPT_TMP zips.
6. **Attribute work:** Commit with descriptive messages so the other model can see
   what moved.
7. **Escalate to the user** for motor/ESC output, arming, or safety-policy changes.
