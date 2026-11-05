# Sources, artifact provenance and reproducibility

## Primary bundles used to author this knowledge base

1. `IntelQCopter_500Hz_Fast_Calibrated_FC_v2.zip` — current source/readme/config, PID model, estimator, tests, CLI scripts.
2. `IntelQCopter_Calibration_20261104.zip` — user-supplied 4 Nov raw CSV, calibration JSON, plot, script and skipped-stage record.
3. `IntelQCopter_Independent_Raw_Calibration_Audit_20261104.zip` — independent numerical re-analysis (including prior captures, candidate JSON and generated reports).
4. `IntelQCopter_100Hz_Original_v13_Driver_Full_Calibration.zip` — working MAG driver and complete calibration tooling.
5. `IntelQCopter_Complete_AI_Handoff_2026-11-03.zip` — early history, v13-v18 evidence, mass model and reference sources.
6. `IntelQCopter_Volume_I_Study_Edition2.pdf` — user-supplied mathematical study; theoretical source only for a number of physical assumptions.

## Remote version control

Public reference: <https://github.com/BenMohStem/IntelQCopter>. As checked on 5 Nov, the remote `main` head was `e2fa10862b0c`; `esp32-xbox` head `af29870011ac`; `codex/qmc-200hz-continuous-diagnosis` head `7833cf87adca`. The supplied v2 ZIP is a **separate local artifact** and should not be assumed present in `main` or deployed to the board. Requery these SHAs before relying on them in the future.

## Reproduce data analysis

Keep raw inputs immutable and compute derivatives in a new directory. The independent audit bundle documents `run_independent_audit.py`, `create_outputs.py`, and `validate_outputs.py`. Compare engineering-unit CSV fields against raw integer axes and firmware sensitivities; use held-out rotation sessions to detect overfitting.

## Reproduce source tests

From the actual v2 folder:

```powershell
py -3 tests/test_project.py
py -3 tools/uncertainty_pid_v2.py
py -3 tools/simulate_rate_v2.py
```

Tests for a simulated/mocked C++ environment cannot establish ESP32 electrical timing or safe motor actuation. Do not modify the calibration JSON in place without recording its source and re-running diagnostics.
