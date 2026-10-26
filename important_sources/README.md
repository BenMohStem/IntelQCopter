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

10. [Current PCB, native KiCad files and print masks](../hardware/home-etched-v3/lidar-gap-two-ground-wires/README.md), [G1/G2 solder guide](../hardware/home-etched-v3/lidar-gap-two-ground-wires/output/assembly-guide.html). The two GND wires are mandatory; current CAD checks pass.

11. [Pin capability audit and firmware integration limits](../hardware/home-etched-v3/lidar-gap-two-ground-wires/pin-audit.md), [prepared pin map](../config/sensor-wiring.json).

12. Historical references: [module palette](../hardware/module-library/QUICKSTART.md), [unrouted 17-part reference](../hardware/connected-i2c/README.md), [old 27-link revision](../hardware/home-etched-v2/README.md), [failed one-jumper checkpoint](../hardware/home-etched-v3/checkpoint-one-jumper/README.md). Their masks are not the current design.

13. [View the current PCB in 3D](../hardware/home-etched-v3/lidar-gap-two-ground-wires/models3d/README.md), [native top render](../hardware/home-etched-v3/lidar-gap-two-ground-wires/output/3d-top.png). Open the PCB and press Alt+3. Models are illustrative; electrical design unchanged.
