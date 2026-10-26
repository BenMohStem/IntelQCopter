# Knowledge base bootstrap / coordinator / 2026-10-26

Baseline: a961a8a, codex/stepwise-esp32. No firmware or calibration changes.
Created snapshot, evidence index, five dependency-aware tasks, current decisions,
handoff template and transactional local ownership tool. Validation includes
eight concurrent contenders for one scope, owner/token release, ancestor/case
conflicts and refusal to steal overdue reservations. This is local cooperative
coordination, not distributed enforcement. Physical rotations remain pending.

Initial concurrency-test teardown failed on Windows because SQLite context managers do not close connections. Explicit contextlib.closing fixed the resource lifetime; the same concurrent ownership and overdue-claim assertions were rerun.
