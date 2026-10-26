# 1,000 Hz acquisition and processing stage

**Updated:** real hardware acquisition and observed handler-budget checks now pass after cross-core logging repair. See [actual results](../measurements/mpu6050/2026-10-26-fifo-and-gyro-frame-results/report.md) and [current frame step](gyro-frame-stage.md). Preparation-only statements below preserve the pre-upload checkpoint.

The FC target is now **1,000 Hz**. One new MPU6050 FIFO packet drives one calibrated state update. This prepares the timing/acquisition foundation; attitude fusion, PID and motor outputs are not implemented in this step. Real hardware rate/deadline acceptance is pending a new upload.

## What changed

- Removed the 10 ms delay and per-sample long float telemetry.
- FIFO packet: acceleration XYZ, temperature, gyro XYZ; 14 bytes. DLPF mode 0/divider 7 retain the nominal 1,000 Hz sensor schedule and existing ranges.
- Apply the same accelerometer matrix and gyro rounding remainder. Latest calibrated state is stored every sample, not only when printing.
- Fast calculations use single-precision constants; slow float diagnostics run at 10 Hz.
- Check all offset words at capture boundaries, not in the hot handler. Unexpected changes are detected at the boundary, not guaranteed immediately.
- Check FIFO sources/control readback, overflow/backlog, I2C failures and a 25-second capture timeout. Unknown/calibration-mismatched state stops acquisition. Motor pins remain input pulldowns; no PWM/ESC driver.
- The default runtime continuously services the FIFO. `?` is a bench identity/configuration query and restarts acquisition; `s` starts an isolated 20,000-sample capture; `x` stops acquisition until reset. Queries/resetting FIFO are maintenance interruptions, not flight operations.

## Evidence already obtained

The ESP32 build passes. Forty-one unique Python arithmetic/protocol tests pass. The actual `firmware/main.cpp` also executes in 14 native host cases with fake Arduino/I2C/UART devices: normal operation, retained trims, invalid accel/gyro boot words, failed trim readback, failed FIFO enable, overflow, failed status/packet reads, changed offsets, bus backlog, timeout, abort and timer wrap inside capture.

Three successful host cases each emit 20,000 samples. The real recorder verifies their sequence, payload hash and 200 MCU diagnostic records. This is execution of the actual C++ source under a **host peripheral model**, not ESP32 emulation, silicon validation or measured MCU timing. The model's microsecond numbers must not be reported as hardware results.

## Hardware test after upload

The board was last observed running V2 at about 83.3 Hz. The new source announces `FIFO1000_V1`. No new firmware has been flashed by the assistant. The user previously reserved uploading; a separate explicit upload-choice question is pending.

After the new firmware is uploaded, the assistant can run:

```powershell
.\qcopter.ps1 record-1khz --port COM3
```

Each run lasts about 20 seconds and saves original serial bytes, all post-trim raw samples, source/configuration metadata, checksum and timing. Capture is rejected for missing/reordered/duplicate/saturated samples, checksum mismatch, missing/incorrect MCU diagnostics, or firmware-reported failure. Originals and failed trials remain local and unchanged.

`S` records carry sequence plus seven raw words. `D` records at every 100th sample carry calibrated acceleration, gyro and gravity angles computed on the MCU. The PC checks them against the exact preceding raw packet. This validates computation without requiring a long float line on every sample.

## What timing results mean

The footer reports sample count, elapsed MCU time, maximum FIFO occupancy, maximum sample-service gap, maximum calculation time, maximum complete sample-handler time and raw-payload FNV-1a checksum. This checksum checks transmission consistency; it is not cryptographic sensor authentication.

Average acquisition rate is count / elapsed time. A provisional +/-2% flag checks average rate sanity. Handler time below 1,000 us is reported separately. Neither flag proves strict real-time PID operation: maximum service gap, FIFO latency/backlog, periodic checks, UART/RTOS behavior and future control work must also fit the budget. Calculation timing excludes I2C and logging; full handler timing includes them. Do not label average throughput as a tested control deadline.

The FIFO clock is the MPU clock, not a calibrated external time reference. Per-sample time reconstructed from sequence is nominal 1 ms spacing; do not invent measured timestamps for each FIFO packet. Counters, no-overflow evidence and elapsed time must be checked together.

## Commands for offline verification

```powershell
.\qcopter.ps1 check-all --native
.\qcopter.ps1 build
```

The check command never opens COM3 or uploads. It preserves original logs under artifacts/bench-checks and artifacts/native-fifo-tests. ESP32 compiler evidence remains under artifacts/build. Only reviewed summaries are published.

## Next gates

1. Upload the prepared bench source, then measure actual 1 kHz throughput, FIFO backlog, service gaps and handler budget. Run captures automatically and preserve failures.
2. Correct any timing/I2C/transport failure before adding control work.
3. Verify gyro-to-drone axes using physical known rotations before integration. A general accelerometer calibration matrix must not be blindly used as a gyro rotation.
4. Add a small attitude filter, then rate/angle PID stages with timing regression. BMP280/altitude remain deferred until available. Arming stays disabled.
