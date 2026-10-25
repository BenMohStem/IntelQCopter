# Current important files

1. [main.cpp - full coupled XYZ calibration](../firmware/main.cpp).
2. [Math, register rounding and live recording guide](../docs/full-accelerometer-calibration.md).
3. [Current plan](../docs/PLAN.md).

[Six-pose fitted model](../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md) · [Independent tilted-pose review](../measurements/mpu6050/2026-10-25-tilt-validation/report.md) · [Raw source checkpoint](../archive/bench-checkpoints/raw-pose1000/README.md).

The historical [X-only register lesson](../docs/bench-x-bias.md) and [value-provenance report](../docs/bias-x-provenance.html) remain explanatory history. The active firmware corrects all axes together. PC tooling is under tools/calibration; it is not flight code.

[First actual full-XYZ recording: report and graph](../measurements/mpu6050/2026-10-25-first-live-xyz/report.md). One implementation run verified; reference accuracy and repeated-boot behavior remain separate checks.
