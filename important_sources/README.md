# Important sources

1. [main.cpp - only active MCU runtime](../firmware/main.cpp).
2. [Current gyro-frame step](../docs/gyro-frame-stage.md) and [1 kHz acquisition](../docs/fifo1000-stage.md).
3. [Short current plan](../docs/PLAN.md).
4. [1 kHz capture/verifier](../tools/calibration/record_fifo1000.py).
5. [One-command offline checks](../tools/validation/run_bench_checks.py).
6. [Actual-source host fault tests](../tools/validation/native_fifo_test.cpp).
7. [Accelerometer math](../docs/full-accelerometer-calibration.md) and [gyro correction](../docs/gyro-bias-application.md).

Canonical links, no duplicate runtime copies. Current source is flashed and tested on hardware; host/model results are separate. Known physical rotations are deferred by the user. Original raw evidence stays private. Motor output remains disabled.
