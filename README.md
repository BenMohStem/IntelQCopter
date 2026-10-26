# IntelQCopter - stepwise ESP32 flight controller

**Target: 1,000 Hz standalone FC.** Current step: calibrated MPU6050 FIFO acquisition/state processing. One Arduino main.cpp; no motor output. The new code builds and passes offline/source-execution tests; physical 1 kHz validation awaits upload.

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

Original samples/logs, MCU computation checks, checksum, throughput, FIFO occupancy and timing are saved automatically. The program rejects corrupt/incomplete capture rather than inventing samples. The previously connected V2 firmware still runs at about 83 Hz until uploaded; no new upload has been performed by the assistant.

## Preserved work

[Accelerometer calibration](docs/full-accelerometer-calibration.md) · [Gyro bias restoration](docs/gyro-bias-application.md) · [Actual gyro restart results](measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md) · [Actual repeatability results](measurements/mpu6050/2026-10-26-sampling-and-repeatability/report.md).

BMP280/altitude are deferred. Gyro frame validation, attitude fusion, PID/mixer and simulated gains are later stages. Physical arming remains disabled. Pico firmware stays on the original main branch; this branch preserves the ESP32 software lessons.
