# BMP280 baseline, noise and zero altitude

## November 4 stationary capture

1,000 accepted barometric samples. Mean pressure approximately **98,382.027 Pa**, stationary pressure SD **0.63965 Pa**, and observed relative-altitude SD **0.05484 m**. These describe a particular stationary test and sensor/filter setting, not a universal pressure sensor absolute-height accuracy specification.

## Startup sequence

1. Read and validate BMP identity and factory trim.
2. Configure normal mode and a bounded wait for stable readings.
3. Collect stationary, fresh pressure observations and compute mean P0, SD and sample count.
4. Set relative altitude zero to P0; initialize altitude-estimator height state and covariance consistently.
5. Reject resets while commanded/estimated motion is significant, and do not accept disconnected sensor zeroing.

## D-pad altitude re-zero

`Up + LB + RB` on the Xbox bridge requests disarmed-only re-zero. Only a fresh, validated pressure window should update P0. Never let a button-press while airborne instantly shift the reference; it would create an uncontrolled height error and PID transient.

## Kalman model tuning

FC v2 uses **0.22 m** provisional barometric measurement SD, deliberately larger than 0.055 m stationary, to reflect unmeasured flow/temperature/propwash effects. This is not an experimentally fitted dynamic variance. Check innovation residuals, temperature and pressure port placement before decreasing the assumed R. LiDAR, when validated, can independently stabilize near-ground relative height.
