# Gyro restoration after reset and full power cycle

The user completed explicitly labeled recordings after an ESP reset and after full power cycling. Original hashes, both headers, source identity, board/profile continuity, every selected original sample, timestamp coverage and corrected-output arithmetic passed offline verification. No serial port was opened or firmware uploaded by the assistant.

| Operator action | Samples | Duration | Full-run mean X/Y/Z (degrees/s) | Gyro target readbacks |
|---|---:|---:|---|---|
| After reset | 2501 | 30.000 s | -0.020958, -0.007900, +0.005865 | -6, +9, +27 |
| After power cycle | 2502 | 30.012 s | -0.002891, +0.001755, +0.016152 | -6, +9, +27 |

Both captures use the same V2 source c145046a129396ee1b49efe1c580bdc842048667eaa1ff66936473ab7a2cc9c0. Boot gyro words were [0,0,0]; firmware restored the expected targets, with no accumulation observed. Accelerometer calibration metadata also agrees. Full covariance, temperatures, drift and original log hashes are in [analysis.json](analysis.json).

This supports restoration in these two operator-performed cases. Supply removal was not independently measured; MCU setup resets the MPU too. This is not proof of all electrical cases, zero drift or exact calibration at all temperatures. The changed residual after the power cycle remains recorded; no new bias is fitted from these validation runs.

![Actual restart recordings](corrected-gyro.png)

The firmware can progress to a gravity-only roll/pitch observation lesson. BMP280 is unavailable and deferred. No altitude estimate, gyro-body-axis integration, Kalman filter, PID or motor output is accepted by these tests.
