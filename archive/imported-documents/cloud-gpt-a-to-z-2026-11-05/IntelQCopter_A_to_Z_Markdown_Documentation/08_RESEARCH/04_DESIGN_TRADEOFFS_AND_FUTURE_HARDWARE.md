# Performance tradeoffs and future design options

## MPU and bus

An MPU6050 is usable for this bench but is limited by shared I²C bandwidth and register-level FIFO behavior. A future SPI IMU with data-ready and hardware timestamps could reduce bus contention and measurement age, if the board is redesigned and tests prove the improvement. Do not redesign hardware before measuring the present `FC_TIMING` bottlenecks.

## Magnetometer

The QMC5883P can be genuinely fresh near 200 Hz in one demonstrated capture, but its board-specific I²C mode failures make it a higher risk than raw datasheet ODR suggests. A magnetometer on a short, remote, shielded mounting location might reduce motor field interference; actual location and current response need measurement.

## Altitude

Fast BMP normal-mode ×1/×1 with IIR×4 can give roughly 166 Hz, but true vertical accuracy depends much more on flow shielding, gyro attitude, acceleration-bias tracking and ground range reference. Optional F2 LiDAR is valuable near ground when verified. Better barometric processing is not equivalent to 200 independent pressure samples/s.

## Communication

921600 CH343 binary capture is adequate for historical 52-byte/500 Hz telemetry under good host conditions, but Windows control/diagnostic tools contend for one COM port. Future flight firmware needs separate reliable RC/radio authority, failsafe and telemetry priorities; Xbox USB bridge is lab tooling.

## Controller architecture

Keep cascade PID as the first learning/validation target; use quaternion geometry and conditional linear altitude KF. Consider model feedforward, LQR, system identification and extended estimators only after physical plant and noise measurements justify the complexity.
