# Acceptance and repair

`qcopter.ps1 status` derives current state from source hashes, configuration, evidence and upstream acceptance receipts. The ledger is append-only and hash-chained; Git additionally preserves revisions. It detects edits but is not a trusted hardware instrument or cryptographic operator identity system.

## Record observations

Build output belongs under `artifacts/build/`. A successful build event does not advance physical acceptance. Simulator evidence names the actual engine and cannot substitute for required physical evidence.

After actually performing a physical procedure, fill the bench identity and module identity in `config/hardware.json`. Then record the result, for example:

```powershell
.\qcopter.ps1 gate record 00-inspection --kind physical --result pass --evidence artifacts/hardware/YOUR-RUN/inspection.csv --note "Actual readings and acceptance review" --operator YOUR-NAME --hardware-id YOUR-BENCH-ID --attest-real-hardware
```

Replace placeholders with real values. Do not execute this example as an unperformed test. Physical pass is refused unless prerequisites have current acceptance receipts. The software cannot verify an operator's claim that a measurement is real; attach original instrument/serial data and review it.

## A failed step

Record `--result fail` with original evidence. The failed stage and dependent acceptances are blocked. Fix the source/configuration in a new commit, rebuild, then repeat real tests in dependency order. A new parent pass does not restore old child passes; each child needs a new receipt.

Keep failed later code in Git history. Remove or replace it in the active build using descriptive new commits when its assumptions were wrong. Do not delete evidence, rewrite shared history or automatically revert unrelated work. For a larger restart, create a repair branch at the last accepted commit, replay corrected work step by step, and retain the rejected branch as history.

Source/config/evidence edits invalidate old acceptance. Changing a stage's prerequisite policy also changes its fingerprint. A calibration/configuration change should be reflected in the tracked inputs rather than hidden as a local tweak.

The gate's unit tests use temporary **synthetic** evidence and never touch the project ledger. Passing them validates gate behaviour only.
