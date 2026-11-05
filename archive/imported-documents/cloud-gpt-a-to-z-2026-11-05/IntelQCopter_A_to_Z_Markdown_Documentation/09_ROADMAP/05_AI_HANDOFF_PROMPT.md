# Instructions for another engineering AI

You are joining **IntelQCopter**, a custom ESP32-S3 F450-class quadcopter project. Read `00_START_HERE/00_README_FIRST.md`, `01_CURRENT_STATUS.md`, `02_EVIDENCE_AND_CONFIDENCE.md`, all of `03_CALIBRATION/`, `04_FIRMWARE/03_FAST_FC_V2_ARCHITECTURE.md`, `05_ESTIMATION/`, `06_CONTROL/`, and `07_TESTING/` before editing firmware.

## Current ground truths

- Actual MAG is QMC5883P `0x2C`, ID `0x80`.
- Successful November 4 full capture had MAG `0x3D`, ~195 Hz; isolated v13 `0xC9` 100 Hz also worked on another run.
- Use November 4 independent numerical candidate constants, not the original degenerate MAG ellipsoid output.
- Current FC v2 is **motor-disabled**; host tests do not establish actual board timing.
- Mass, inertia and motor thrust/lag uncertainty are assumption-based.
- Direct Xbox BLE and ESP32 LiDAR RX are unverified; virtual Xbox/CH343 bridge only.

## Your work contract

1. Work on a new Git branch/versioned project folder, preserving source artifacts and immutable raw data.
2. State exactly what you changed, expected impact and why measured evidence supports it.
3. Maintain `tools/` PowerShell entry points, CH343 autodetection and Windows-friendly workflow.
4. Keep `IQ_MOTOR_OUTPUTS_ENABLE=0`; do not enable ESC outputs or remove failsafe barriers.
5. Run host tests plus actual PlatformIO build where available, then ask the human for a motor-off hardware log; label what you did not run.
6. Evaluate numerical residuals, 500 Hz worst-case timing, FIFO health, magnetometer freshness and barometer KF NIS rather than average-only metrics.
7. Report formulas, dimensions, uncertainties and physical limitations; do not claim flight readiness.

You can develop a fault-injection suite or better model identification analysis now, without inventing a motor thrust curve or conflicting with the other AI’s COM port session.
