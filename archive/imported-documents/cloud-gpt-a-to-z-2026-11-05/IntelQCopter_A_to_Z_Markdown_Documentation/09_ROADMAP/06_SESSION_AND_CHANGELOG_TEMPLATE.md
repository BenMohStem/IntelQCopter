# Reusable experiment template

Copy the following for each firmware change or board test.

```markdown
# IntelQCopter test: <short name>
Date/time and timezone:
Operator:
Git branch + commit / ZIP SHA256:
Power state: cold sensor power cut / ESP reset / USB-only reset
Hardware: board, wiring, LiPo/USB, props removed, ESC signals disconnected
Firmware: environment, flags, QMC C1/C2, MPU DLPF/range/FIFO, BMP mode, I2C Hz, UART baud
Change from baseline:
Hypothesis:
Procedure:
Expected observable success/failure:
Log paths + sizes + checksums:
Metrics: IMU/MAG/BMP fresh rates, loop avg/P99/max us, late/miss,
  fifo loss/overflow, I2C errors, CRC/sequence loss, temperature,
  baro NIS, raw gyro residual and accel g norm.
Safety: physical motor output disabled? verified?
Result: PASS / FAIL / INCONCLUSIVE
Evidence and caveats:
Next experiment:
```

## One-change-at-a-time examples

- `0x3D QMC profile, 400 kHz -> 100 kHz QMC reads` while holding all other settings fixed.
- Boot gyro offset feature off -> on with same sensor temperature and stationary orientation.
- BMP IIR×4 -> ×2 with ruler-controlled step, measuring lag and pressure SD.

For repeated experiments, maintain a `runs.csv` alongside raw logs rather than overwriting `last_run.log`.
