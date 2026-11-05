# Roll/pitch angle and yaw-rate control

## Angle loops

For roll and pitch, the outer PID compares an angle request with quaternion-derived Euler roll/pitch, then requests a body rate. These gains are substantially lower-bandwidth than the inner rate loops. FC v2 clamps commanded roll/pitch to approximately **±15°** during virtual bench operation and outer rate request to configured bounds.

The `PID.step(..., derivativeInput=gyro_axis)` pattern uses measured angular rate as the derivative channel for angle controllers, avoiding derivative kick when setpoint jumps. Confirm gyro and Euler signs match FLU conventions before interpreting PID feedback as negative feedback.

## Yaw

The Xbox left-X input maps to yaw-rate request, clamped in v2 near **±1.4 rad/s**. There is no yaw-angle PID, which avoids imposing an absolute heading when MAG mounting/ref is unverified. Slow magnetic yaw correction is in the estimator only; if MAG trust fails, yaw angle drifts but rate control can still be evaluated in a safe simulator.

## Required experiments

With motors disabled, hand-rotate aircraft while holding roll/pitch setpoints at zero; verify virtual torque tends to oppose measured motion in all axes. Cross-axis responses must not change sign. Only after physical axis mapping and actuator measurement can gains be related to actual rotor thrust.
