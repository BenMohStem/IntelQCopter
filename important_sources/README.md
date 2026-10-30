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


## Current operator revision - 30 October 2026

[Small Xbox input-only main.cpp](../lessons/xbox-input/main.cpp), [PlatformIO configuration](../lessons/xbox-input/platformio.ini), [revision guide](../docs/revision-and-xbox.md). Temporary independent lesson; replaces running firmware only when YOU upload it. No motors/PID; original MPU runtime remains in firmware/main.cpp. [Battery-current sensor choices](../docs/battery-current-sensor.md).


## Real acquisition comparison - 30 October 2026

[Actual results and analysis lesson](../measurements/mpu6050/2026-10-30-acquisition-comparison/report.html), [Markdown](../measurements/mpu6050/2026-10-30-acquisition-comparison/report.md), [12-run timing CSV](../measurements/mpu6050/2026-10-30-acquisition-comparison/runs-summary.csv), [separate motor-disabled measurement main.cpp](../lessons/imu-acquisition-benchmark/main.cpp). Board now runs this benchmark; original16MBflash backup is private. Production MPU source/calibration unchanged. No production direct/FIFO choice or flight acceptance implied.

## Current installed bench firmware — 30 October 2026

[Two-core Xbox menu main.cpp](../lessons/xbox-test-menu/main.cpp), [recording instructions and actual results](../measurements/software/2026-10-30-xbox-test-menu/README.md). Supersedes the earlier installed-benchmark statement above. Four serial-requested captures passed; Xbox disconnected, physical buttons pending. No PID/motor output.

Latest Xbox menu/LED instructions: [connection indicator and simpler test selection](../measurements/software/2026-10-30-xbox-led/README.md). Supersedes earlier Menu-only interaction instructions.

[Next step: physical Xbox input mapping and why acquisition tests were needed](../measurements/software/2026-10-30-xbox-mapping/README.md). Includes monitor-only command; Menu/Y correction is controller-specific.

[Operator Xbox input results and future battery LED requirement](../measurements/software/2026-10-30-input-results/README.md). No firmware/actuator changes.

[Next: one labelled positive-X gyro direction recording](../docs/gyro-single-check.md). No firmware upload or calibration change.
