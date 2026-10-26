# Multi-agent work protocol

## Ownership and integration

The coordinator splits work into small tasks with explicit file scopes. Different
scopes can run in parallel. Treat firmware/main.cpp, a serial device, shared
catalogs and Git publication as exclusive resources. Reserve before editing;
read-only reviews need no claim. File scopes are case-insensitive path prefixes.

`claim` atomically checks and inserts all scopes using SQLite BEGIN IMMEDIATE.
A task must be ready and dependencies done. The token belongs to its owner;
release requires both. The review time is a reminder, not permission to steal
work. Even overdue claims remain blocking. If an owner disappears, the coordinator
must confirm that work stopped and inspect its diff before explicitly recovering
the reservation; there is intentionally no automatic takeover command.

Use separate Git worktrees for broad changes; all worktrees share this local
reservation DB. In a shared checkout, workers must not switch branches, reset,
stash others' changes, or stage unrelated files. One integrator commits/pushes;
workers hand off exact paths, tests and limitations. If another tool already
holds COM3, respect that ownership too.

These are cooperative reservations, not OS locks or access control. Agents can
bypass them by ignoring the protocol. Separate clones/computers do NOT share this
DB: use one coordinator assigning disjoint tasks across machines. Never claim
this provides distributed locking or prevents every filesystem race.

## Before completion

1. Inspect the diff and run checks appropriate to the change.
2. Write a unique handoff using handoffs/TEMPLATE.md; identify engine, source,
   result, assumptions, failures and remaining physical gates.
3. Ask the integrator to review. A worker's successful check is not automatic
   hardware acceptance. Update task status and relevant facts only after review.
4. Release ownership after handoff/review is coordinated, not while still editing.
5. Integrator checks knowledge freshness, commits explicit paths and pushes.

Knowledge contains factual summaries and concise rationale. Keep credentials,
personal data, raw hardware identifiers and private recordings out of publishing.
Link reviewed evidence; retain raw originals privately, including failed trials.
