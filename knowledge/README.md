# Project knowledge index

Use this index and project.json instead of rereading the full conversation or historical reports.

| Record | Where |
|---|---|
| Architecture and controls | ../README.md |
| Build/wiring/calibration | ../docs/setup.md |
| Requirements and remaining work | ../docs/status.md |
| Facts, decisions, assumptions and sources | project.json |
| Searchable factual database | project.sqlite |
| Source-project review | ../docs/drone-review.md |
| Historical evidence and implementations | ../archive/ |

Update rules:

1. Label facts confirmed, implemented, assumed, measured, historical or pending.
2. Attach a path, commit, user decision or original log.
3. Separate a compile from physical measurement and flight evidence.
4. Supersede statements explicitly; preserve original evidence.
5. Never record secrets or invented measurements.

This is a factual ledger and archive index, not an unstructured dump of old plans.

After editing project.json, run python tools/update_knowledge.py.
Example SQL: SELECT id, statement, source FROM facts WHERE status = 'pending';
