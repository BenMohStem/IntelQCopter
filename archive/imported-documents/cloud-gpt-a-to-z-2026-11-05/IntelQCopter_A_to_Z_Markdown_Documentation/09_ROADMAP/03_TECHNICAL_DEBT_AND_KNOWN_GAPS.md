# Technical debt and integration gaps

## Highest-risk

- Physical safety: ESC output driver, arming, independent kill, lost-link policy and motor order not validated.
- Estimator frames: sensor-to-body matrices still identity placeholders in v2.
- Vertical control: no validated LiDAR integration and unmeasured dynamic BMP pressure disturbance.
- Dynamics: motor thrust curve, lag, yaw torque and actual COM/inertia missing.
- FIFO and I²C: 400 kHz integration off-device tested but not benchmarked on board.

## Source-level cleanup candidates

- Separate sensor HAL from controller core, keep a stable binary sensor capture/diagnostic mode.
- Add version/hash to boot output and structured logs with timestamps and units.
- Introduce single authoritative mode/command schema across historical ESP32/Xbox branches.
- Make fault state machine explicit with no automatic recovery to armed after any critical fault.
- Add deterministic unit tests for motor allocator saturation, dt outliers, power-fault response, sensor frame transforms and magnetometer norm gates.
- Validate pressure compensator numerics on several operating points and prove read-scheduling freshness.

## Calibration pipeline

Replace the old degenerate MAG ellipsoid fitter, remove misleading plotted “corrected” shape, and store candidate parameter confidence/provenance in one signed/versioned artifact. Revalidate full accel matrix with better face fixture and gyro drift with temperature logs.

## Avoid false optimization

Do not chase the fastest apparent average sample rate if the real worst-case delay, sensor covariance, actuation latency or reliability becomes worse.
