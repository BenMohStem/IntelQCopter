# Important sources

1. [main.cpp - only active MCU runtime](../firmware/main.cpp).
2. [Current gyro-frame step](../docs/gyro-frame-stage.md) and [1 kHz acquisition](../docs/fifo1000-stage.md).
3. [Short current plan](../docs/PLAN.md).
4. [1 kHz capture/verifier](../tools/calibration/record_fifo1000.py).
5. [One-command offline checks](../tools/validation/run_bench_checks.py).
6. [Actual-source host fault tests](../tools/validation/native_fifo_test.cpp).
7. [Accelerometer math](../docs/full-accelerometer-calibration.md) and [gyro correction](../docs/gyro-bias-application.md).

Canonical links, no duplicate runtime copies. Current source is flashed and tested on hardware; host/model results are separate. Known physical rotations are deferred by the user. Original raw evidence stays private. Motor output remains disabled.

8. [Offline angle filter and DOE guide](../docs/attitude-preparation.md), [small PC candidate](../tools/estimation/prepare_attitude.py), and [results/graphs](../measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md). Firmware is unchanged; physical reference rotations remain pending.

9. [Project knowledge, evidence and agent coordination](../knowledge/README.md).

10. [KiCad component palette and pin guide](../hardware/module-library/QUICKSTART.md): placed module/header interfaces for user PCB design.

11. [Connected I2C hardware project and wiring](../hardware/connected-i2c/README.md), [pin map](../config/sensor-wiring.json).

12. [Routed single-sided ESP32-S3 PCB, KiCad and three-copy mask](../hardware/home-etched-v2/README.md); [interactive assembly guide](../hardware/home-etched-v2/output/assembly-guide.html).
