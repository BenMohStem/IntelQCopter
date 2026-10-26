# IntelQCopter - stepwise ESP32 flight controller

**Target: 1,000 Hz standalone FC.** Current step: calibrated MPU6050 FIFO acquisition/state processing. One Arduino main.cpp; no motor output. Actual 1 kHz throughput and observed handler-budget tests pass after cross-core logging repair. FIFO1000_V2 also computes provisional body-frame gyro rates; known physical rotations remain pending.

[Current source](firmware/main.cpp) · [1 kHz stage and verification](docs/fifo1000-stage.md) · [Short plan](docs/PLAN.md) · [Important sources](important_sources/README.md).

## Checks

```powershell
.\qcopter.ps1 check-all --native
.\qcopter.ps1 build
```

These do not open the board or upload. Host tests execute actual firmware with fake peripherals; they are not ESP32 emulation or physical timing evidence.

## Real acquisition

After the new bench firmware has been uploaded, the assistant can record and verify 20,000 samples per run:

```powershell
.\qcopter.ps1 record-1khz --port COM3
```

Original samples/logs, MCU computation checks, checksum, throughput, FIFO occupancy and timing are saved automatically. The program rejects corrupt/incomplete capture rather than inventing samples. The assistant uploaded and tested current source with explicit permission. Captures run near 1000.69 Hz; inter-service jitter remains, so no strict periodic or full-FC readiness claim.

## Preserved work

[Accelerometer calibration](docs/full-accelerometer-calibration.md) · [Gyro bias restoration](docs/gyro-bias-application.md) · [Actual gyro restart results](measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md) · [Actual repeatability results](measurements/mpu6050/2026-10-26-sampling-and-repeatability/report.md).

BMP280/altitude are deferred. Gyro frame validation, attitude fusion, PID/mixer and simulated gains are later stages. Physical arming remains disabled. Pico firmware stays on the original main branch; this branch preserves the ESP32 software lessons.

[Actual FIFO timing and preserved failures](measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md) · [Current gyro-frame step](docs/gyro-frame-stage.md).

[Offline angle-filter preparation: equations and fewer-test design](docs/attitude-preparation.md) · [Results and graphs](measurements/mpu6050/2026-10-26-offline-attitude-preparation/report.md). Reuses 80k saved samples; four modeled combinations expose acceleration sensitivity. No MCU filter or acceptance change.

[Multi-agent knowledge base: start here](knowledge/README.md).

Current PCB: [lidar gap, two GND wires and pin audit](hardware/home-etched-v3/lidar-gap-two-ground-wires/README.md). Native DRC, connectivity and schematic parity pass; fit G1/G2 and perform physical checks before use.

[Historical CAD](hardware/home-etched-v3/README.md) is retained for traceability; use the current revision's matching masks.
