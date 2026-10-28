# Important sources

1. [main.cpp - only active MCU runtime](../firmware/main.cpp).
2. [Current gyro-frame step](../docs/gyro-frame-stage.md) and [1 kHz acquisition](../docs/fifo1000-stage.md).
3. [Short current plan](../docs/PLAN.md).
4. [1 kHz capture/verifier](../tools/calibration/record_fifo1000.py).
5. [One-command offline checks](../tools/validation/run_bench_checks.py).
6. [Actual-source host fault tests](../tools/validation/native_fifo_test.cpp).
7. [Accelerometer math](../docs/full-accelerometer-calibration.md) and [gyro correction](../docs/gyro-bias-application.md).

Canonical links, no duplicate runtime copies. The prior FIFO source was flashed/tested; the latest startup guard is build/host-tested only, pending hardware upload. Host/model results remain separate. Known physical rotations are deferred by the user. Original raw evidence stays private. Motor output remains disabled.

8. [Offline angle filter and DOE guide](../docs/attitude-preparation.md), [small PC candidate](../tools/estimation/prepare_attitude.py), and [results/graphs](../measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md). Firmware is unchanged; physical reference rotations remain pending.

9. [Project knowledge, evidence and agent coordination](../knowledge/README.md).

10. [Current PCB, native KiCad files and print masks](../hardware/home-etched-v3/dedicated-ground-jumpers/README.md), [G1/G2/G3 + RF-clearance guide](../hardware/home-etched-v3/dedicated-ground-jumpers/output/assembly-guide.html). Three wires are mandatory; G2 now has a third dedicated RF-return tap. Current CAD checks pass.

11. [Pin capability audit and firmware integration limits](../hardware/home-etched-v3/dedicated-ground-jumpers/pin-audit.md), [prepared pin map](../config/sensor-wiring.json).

12. Historical references: [module palette](../hardware/module-library/QUICKSTART.md), [unrouted 17-part reference](../hardware/connected-i2c/README.md), [old 27-link revision](../hardware/home-etched-v2/README.md), [failed one-jumper checkpoint](../hardware/home-etched-v3/checkpoint-one-jumper/README.md). Their masks are not the current design.

13. [View the current PCB in 3D](../hardware/home-etched-v3/dedicated-ground-jumpers/models3d/README.md), [native top render](../hardware/home-etched-v3/dedicated-ground-jumpers/output/3d-top.png). Open the PCB and press Alt+3. Models are illustrative; six dedicated jumper holes added; original module/header interfaces unchanged.

14. [Latest minimal input guard and test evidence](../measurements/software/2026-10-27-input-guard/README.md). No motor output; no new hardware test.

15. [Zero-jumper feasibility method and independent CAD gate](../hardware/zero-jumper-study/method/README.md). Tests and negative controls are evidence about the checker; no candidate is accepted merely because it was generated.


## University submission in English

[Professional and Personal Project report](../docs/university-report/technical-report.html) ? [PDF](../docs/university-report/technical-report.pdf) ? [Materials list](../docs/university-report/bill-of-materials.csv). Reviewed draft; see its completion checklist before submitting.
