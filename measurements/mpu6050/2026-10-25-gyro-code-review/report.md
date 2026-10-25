# Gyro code review

The current firmware builds; 15 offline tests pass (6 acceleration, 9 gyro/recording tests). No upload, serial connection, simulator or physical test was performed.

The 14-byte register burst order, signed conversion, 65.5-count degree/s conversion and temperature conversion agree with the configured protocol. Full accelerometer correction remains unchanged. Gyro offset registers are read only. All four motor pins remain inputs with pulldowns.

The recorder previously relied on host elapsed time and a minimum sample count. A stream that stopped early could incorrectly satisfy that count. It now checks device-time coverage (90-110% of requested duration), rejects duplicate/backward timestamps and gaps over 100 ms, and records actual duration/largest gap. Constant timestamps cannot produce a drift estimate. Regression cases cover truncated capture, gaps, reset, duplicate timestamps and normal millis wrap. Both query attempts are now listed in the command audit.

Your three `{ 0 }` initializers are retained; they are equivalent zero initialization. No functional firmware changes were needed in this review. Their source fingerprint differs, so upload the current bench source yourself before recording.

**Gyro calibration is pending and required before attitude estimation.** No current gyro recording was available. Do not use invented or pre-remount bias values. The next real measurement is the stationary capture in [the gyro guide](../../../docs/gyro-observation.md). Then review bias, covariance, temperature/drift and the held-out residual, apply the accepted correction exactly once at boot, and verify fresh corrected rates after restart. A static test does not validate rate scale or body-frame mapping.

Original compiler output remains in the local artifacts/build folder named in [verification.json](verification.json). This review supports a bench measurement step; it does not establish physical accuracy or flight readiness.
