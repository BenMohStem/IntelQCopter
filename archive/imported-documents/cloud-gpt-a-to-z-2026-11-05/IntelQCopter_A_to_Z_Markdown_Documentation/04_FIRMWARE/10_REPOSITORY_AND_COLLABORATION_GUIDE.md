# How two AIs and the project owner should collaborate on firmware

## Branch and hardware ownership

- Only one process can own the ESP32's COM port or flash at once. Close GUI, monitor, pyserial recorder, Xbox bridge, and other IDE monitors before switching.
- Each agent works on a separate branch/worktree; avoid editing the same `src/main.cpp` without shared diffs.
- Preserve a tag/ZIP of successful 4 Nov sensor source and raw captures before any FIFO/filter refactor.

## Every change should include

`Commit/patch ID; source base; changed files; changed register values; build flags; local/sim tests; board power state; UART port/baud; hardware test; before/after metrics; remaining unknowns`.

## Review requirements

- Model changes: verify units, sign conventions, solver assumptions and physics dimensions.
- Calibration changes: verify source raw sessions, fit and holdout, hardware offset/matrix application order and actual register readbacks.
- Timing changes: show actual worst-case loop and I²C latency, not just a CPU benchmark.
- Controller changes: compare 7 gains, anti-windup, allocation saturation, step response and uncertainty; never assert stability from a single synthetic trajectory.
- Motor output changes: independent safety, power, failsafe, direction and thrust evidence. The present FC build must remain motor-disabled.

## Good working pattern

One AI owns a small, precise source patch; the other reviews tests and raw logs. Users choose which verified branch to flash. Documentation should remain updated whenever source changes, and the final build should print a version identifier/hash at boot.
