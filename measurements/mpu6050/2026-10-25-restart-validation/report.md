# Reset and power-cycle restoration verified

Engine: offline Python/NumPy/Matplotlib verification of original real ESP32-S3/MPU6050 serial recordings. No simulator, new firmware upload or assistant serial-port connection. The user performed the physical actions and labeled the runs; electrical supply removal was not independently instrumented.

## Actual recorded results

| Operator run | Samples checked | Rate (Hz) | Startup X/Y/Z | Applied X/Y/Z | Maximum arithmetic difference (m/s²) |
|---|---:|---:|---|---|---:|
| initial run | 2727 | 90.906 | [-2074, 2979, 1074] | [-2348, 2993, 1212] | 0.000001542 |
| after ESP32 reset | 910 | 90.900 | [-2074, 2979, 1074] | [-2348, 2993, 1212] | 0.000001481 |
| after full power cycle | 910 | 90.900 | [-2074, 2979, 1074] | [-2348, 2993, 1212] | 0.000001498 |

Both new 10-second recordings contain 910 samples (1820 checked in total). Original-file hashes, source/configuration identity, same-board header, fitted coefficients, sample timestamps, saved-sample equality and every XYZ matrix calculation were rechecked. The source fingerprint matches the current compiled lesson. No coefficients were refitted or modified.

## What this demonstrates

After both operator-performed restart cases, the observed startup words were the recorded baseline values. The firmware restored the same absolute targets and verified them before reporting acceleration. X did not progress to -2622, Y did not progress to 3007, and Z did not progress to 1350, which would be another round of the same trim decrement. The same targets remained checked while samples were emitted. No accumulated correction is observed in these runs.

This directly addresses the permanent-storage question: the custom writable MPU words did not retain our target through this observed initialization sequence; the firmware successfully restored the desired values from the calibration retained in ESP32 program flash. The MCU initialization itself resets the MPU, so these logs do not isolate the effects of an MCU-only reset from the MPU reset inside setup, nor do they prove every conceivable restart condition.

## Limits

Passing restoration checks does not certify the true bias, scale or angular accuracy. The earlier independent angle reference was hand-held with an operator estimate of about ±1 degree. The first full-XYZ run was motionless on an unlevel support; the two new runs have no independently measured pose reference. Differences in mean vectors are therefore not automatically drift or a calibration failure. No new level trim, normalization to g, gyro compensation or flight-control acceptance is introduced.

The observed stream is about 90.9 Hz bench telemetry, not 1000 Hz FIFO data or a 500 Hz control loop. All original recordings, including source/offset headers, remain unchanged and private under artifacts/accel-recordings.

## Next lesson

Accelerometer restoration is now verified for these reset cases. The next small capability is gyro XYZ observation and stationary bias calibration. Preserve the full accelerometer matrix and fixed mounting. Zero gyro rate is checked while the assembly is motionless; exact level is not needed. Do not apply the accelerometer gain/shear matrix directly to gyro rates. Motor outputs remain disabled, and no gyro firmware change is made by this review.

## Reproduce the per-run checks

Use analyze-accel on each local session with a fresh output directory. The operator notes, original source and hashes are retained in the local summaries. Do not overwrite earlier analysis folders.

```powershell
.\qcopter.ps1 analyze-accel --session artifacts/accel-recordings/20261025T200450965913Z --output .local/reset-review-new
.\qcopter.ps1 analyze-accel --session artifacts/accel-recordings/20261025T200531245399Z --output .local/power-cycle-review-new
```

[First actual XYZ report and graph](../2026-10-25-first-live-xyz/report.md) · [Current firmware](../../../firmware/main.cpp) · [Full calibration guide](../../../docs/full-accelerometer-calibration.md).
