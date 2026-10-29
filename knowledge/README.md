# IntelQCopter project knowledge base

**Start here when joining the project.** This is a compact store of facts,
decisions, evidence and handoffs, not another copy of all documents or chat.

## Read in this order (about two minutes)

1. [Current state](state.json): branch, restrictions, tested source fingerprints, open gates.
2. [Evidence index](evidence.json): what each result actually demonstrates and its limits.
3. [Decisions](decisions/0001-current-contract.md): current requirements and superseded assumptions.
4. [Task records](tasks/): choose one ready task; respect dependencies and acceptance criteria.
5. [Coordination protocol](PROTOCOL.md): reserve ownership before writing or using hardware.

Use `project.json` / `qcopter.ps1` for commands. Do not assume the default
QCopter checkout is the active Stepwise branch. Do not reread every historical
log to begin a narrow task; follow the relevant evidence link instead.

```powershell
.\qcopter.ps1 knowledge status
.\qcopter.ps1 knowledge search gyro
.\qcopter.ps1 knowledge check
.\qcopter.ps1 knowledge claim offline-estimator --owner agent-alice
.\qcopter.ps1 knowledge claims
# Keep the returned reservation token; release after handing off your changes:
.\qcopter.ps1 knowledge release --owner agent-alice --token YOUR_RETURNED_TOKEN
.\qcopter.ps1 check-knowledge
```

Git stores durable knowledge. A small SQLite database in the Git common directory
stores live reservations shared by this repository's local worktrees. It is
never committed. Searching/checking/status require no database or hardware.
No external service, embeddings, credentials or downloaded dependencies needed.

## Maintenance

- Update the current snapshot when facts change; keep old decisions/evidence.
- Add a numbered decision with `supersedes` for changed policy; do not erase the old one.
- Each worker writes one uniquely named handoff, rather than all workers editing an index.
- Fingerprint mismatch means affected claims need review. Do not blindly refresh hashes.
- When a test fails, mark affected tasks/acceptance invalidated and dependent tasks blocked;
  preserve failed logs and prior commits. Repair on top of the last valid stage.
- Run `knowledge check` before publishing. It checks source fingerprints, evidence paths,
  task IDs/references and dependency cycles. It cannot certify scientific truth.

The latest user instruction and applicable working agreement take precedence.
Historical prose may be superseded; report contradictions instead of silently
using an old calibration, 500 Hz target, upload prohibition or simulated pass.

## Domain catalog (schema2)

Read [the catalog](index.json) and [maintenance policy](DOMAIN-CATALOG.md) for report facts and current presentation preferences. It is additive: state.json, evidence.json, task dependencies and SQLite claims remain authoritative for their runtime/coordination scopes. Run `python tools/knowledge/check_domains.py .`; remote references are not fetched. Do not refresh stale firmware fingerprints to satisfy a report check.
