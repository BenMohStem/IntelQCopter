# Uncertainty-driven fusion: choosing Q, R and confidence gates

A Kalman filter needs physically meaningful process/measurement variance, not arbitrary extremely small R values that make the estimate look smooth. The 4 Nov BMP stationary relative-altitude SD was 0.0548 m, yet FC v2 uses **0.22 m** nominal dynamic measurement SD to allow for airflow and environment. Both numbers must be labeled with their test conditions.

## Recommended validation logic

1. Estimate sensor noise from *independent* stationary and dynamic captures, with timestamps and a documented preprocessing filter.
2. Track innovation `ν=y-Hx_pred` and `S=H P_pred Hᵀ+R` for each sensor.
3. Count NIS gate acceptances and rejections; investigate persistent one-sided residuals as bias/misalignment rather than increasing process noise forever.
4. Repeated cached BMP/QMC values cannot reduce covariance every 2 ms; update only for fresh conversions.
5. For LiDAR, adapt R using validity/signal or documented uncertainty only after confirming physical frame and range.

## Correlated errors

Vertical acceleration and quaternion tilt share the same IMU data; their errors are coupled. Treating them as fully independent overestimates certainty. Pressure drift is temporally correlated. The simple conditional linear KF in v2 is valuable but not a full vehicle navigation covariance system.

## Robustness

On MAG loss, preserve gyro-rate roll/pitch control and mark yaw heading untrusted; on BMP invalidity, prevent altitude-hold virtual demand from pretending a verified height; on stale pilot UART, invalidate commands. In the current bench build all motor demands remain virtual anyway.
