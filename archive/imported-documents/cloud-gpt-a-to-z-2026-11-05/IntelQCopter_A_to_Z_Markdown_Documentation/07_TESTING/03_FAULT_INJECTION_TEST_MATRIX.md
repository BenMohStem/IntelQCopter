# Fault injection matrix (safe, motor-off)

| Injected condition | Expected response | What to record |
|---|---|---|
| No magnetometer ACK / frozen data | MAG invalid; no forced yaw correction | Age, error count, gyro-only yaw trend |
| QMC read error after config | Bounded retry/fallback, IMU path maintained | Register readback, Wire error, loop duration |
| BMP disconnected or invalid | Height untrusted, altitude PID virtual demands not accepted | `baro0`, `H_BARO_STALE`, KF state |
| Accel saturated | Flag H_ACCEL_CLIPPED, suspend affected estimate/control | Range, raw count, attitude and health |
| Gyro clipped | Flag H_GYRO_CLIPPED; no false precision | Raw p/q/r, integrator reset |
| MPU FIFO overflow | Record FIFO loss and reset boundedly | Frames dropped, loop stalls, sensor age |
| 5–10 ms synthetic task stall | Report missed deadline; no silent time-series compression | FC_TIMING, late/miss |
| Xbox unplugged / serial stopped | Pilot invalid after timeout, virtual outputs zero | link age, parser counters |
| Corrupted/duplicate `$IQ2` packet | CRC/sequence reject | accepted/rejected counters |
| Press altitude-zero chord while stale pressure | Do not accept zero | resetAccepted/Deferred and pressure age |
| Bad mass/inertia matrix | Reject invalid model | `H_MODEL_INVALID`, zero virtual motor |

The present software fault injection is not a substitute for a verified independent physical motor-cutoff architecture. No motor actuation should be enabled to “try” the failure tests.
