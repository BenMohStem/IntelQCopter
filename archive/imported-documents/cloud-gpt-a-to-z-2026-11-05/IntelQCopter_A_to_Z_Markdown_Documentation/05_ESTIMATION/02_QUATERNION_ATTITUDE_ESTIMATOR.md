# Quaternion attitude estimation

v2 stores a body-to-world unit quaternion `q=[w,x,y,z]`, advances it by corrected gyroscope angular velocity, and normalizes each integration. At the start of a stationary session, roll/pitch are initialized from the calibrated gravity vector; yaw remains arbitrary absent a validated magnetic reference.

## Propagation

\[
\dot q=\tfrac12 q\otimes [0,\omega_B],\quad
q_{k+1}=\operatorname{normalize}(q_k+\Delta t\dot q_k).
\]

For the 2 ms loop, rates must be in rad/s and the multiplication order must match body-to-world convention. The current first-order numerical integration is a pragmatic approximation; its error rises with rotation speed and irregular dt. Use recorded timestamps and physical axis tests.

## Correction

The source computes a small proportional correction from the cross product between normalized measured specific force and gravity predicted in the body frame. Acceleration correction is gated by near-1g norm (0.85–1.15g in source), to avoid incorrectly interpreting strong vehicle maneuvers as tilt. A slower magnetic yaw correction is applied only when fresh, plausible MAG data exists, and reference direction is initialized from an initial level-consistent field.

## Not an EKF

This is an integrated quaternion + geometric/complementary correction, **not** a quaternion extended Kalman filter. The altitude subsystem is a separate linear 3-state KF. If maintaining strictly linear control, quaternion geometry remains the kinematics/estimator layer; the control laws can still be cascaded linear PID near hover.

## Failure modes

Non-unit quaternion, clipped gyro, invalid accel orientation, sustained acceleration, magnetometer interference, timestamp gaps and unverified body axes. All require health gating; magnetometer should not be forced into the estimate when corrupted.
