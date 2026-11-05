# Estimator faults, loss of reference and reset policy

The FC `Health` bitmask includes IMU missing, time invalid, gyro residual not accepted, accel bad, barometer not zeroed, pilot stale, attitude bad, model assumed, MAG unverified, clipped gyro/accel, stale pressure, model invalid, rejected Kalman measurements and LiDAR unverified.

## Propeller-free fault injections

- Disconnect MAG or force no fresh status: heading should go untrusted, while gyro/accel logging continues.
- Disconnect barometer: altitude reference becomes unhealthy; no false hold claim.
- Inject a large baro step or LiDAR outlier in host tests: NIS gate increments rejection counter.
- Skip IMU FIFO frames and simulate slow I²C: deadline/freshness counters must expose the problem.
- Feed invalid quaternion/NaN/pilot packets: control core should reset integrators and output zero virtual motors.

## Re-zero policy

Barometer zero is a reference change, not physical motion. It should be allowed only when disarmed, with fresh stable pressure, and must reset altitude KF state and height target consistently. The present source sets `setBaroZero()` and resets PID integrals; controller physical arming is permanently unavailable.

## Future failsafe criteria

Actual motor system needs dedicated state transitions and independent recovery/kill logic; resetting a software health flag is not sufficient. Never permit a transient “all sensors okay for one tick” to auto-arm or resume thrust.
