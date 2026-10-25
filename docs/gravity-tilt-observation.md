# One new idea: roll and pitch from gravity

BMP280 testing is deferred until the sensor is available. This lesson needs only the calibrated MPU6050. The previous gyro reset/power-cycle recordings have been checked: [actual evidence](../measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md). Bias residuals remain documented; no calibration constants changed.

## The small main.cpp change

```cpp
float yz = hypotf(drone[1], drone[2]);
float roll = yz > 0.001f ? atan2f(drone[1], drone[2]) * RAD_TO_DEG : NAN;
float pitch = yz > 0.001f ? atan2f(-drone[0], yz) * RAD_TO_DEG : NAN;
```

`drone[0]`, `[1]`, `[2]` are the existing calibrated accelerometer X/Y/Z in drone coordinates, in m/s². `hypotf(y,z)` calculates sqrt(y²+z²). `atan2f(y,x)` finds the angle of a two-component vector, in radians. `RAD_TO_DEG` converts radians to degrees by multiplying by 180/pi. The small guard marks the roll/pitch pair unavailable near the pitch-90-degree singularity rather than inventing roll there.

Two new columns at the end of the original IMU packet contain these angles, computed by the MCU itself. The header becomes `IMU_XYZ_V3`; the original counts, acceleration, gyro rates, temperature and all correction math remain intact. The PC checks the MCU angle values against the transmitted calibrated acceleration; it does not send flight or calibration commands.

## Why the formulas work when the drone is still

At rest, in the same Z-up calibration convention, the accelerometer measures the gravity-opposing specific-force vector in body coordinates. Let phi be roll and theta be pitch, using R = Rz(yaw) Ry(theta) Rx(phi). Then

\[
\begin{bmatrix}a_x\\a_y\\a_z\end{bmatrix}
=g\begin{bmatrix}-\sin\theta\\\sin\phi\cos\theta\\\cos\phi\cos\theta\end{bmatrix}.
\]

For ordinary upright poses with |theta| < 90 degrees:

\[
\sqrt{a_y^2+a_z^2}
=g\sqrt{\cos^2\theta(\sin^2\phi+\cos^2\phi)}
=g\cos\theta.
\]

Therefore the vector (-ax, sqrt(ay²+az²)) has direction theta, and the vector (az, ay) has direction phi:

\[
\theta=\operatorname{atan2}(-a_x,\sqrt{a_y^2+a_z^2}),
\qquad
\phi=\operatorname{atan2}(a_y,a_z).
\]

No gravity magnitude is needed in the angle ratio. That does not remove the need for calibrated bias, scale and mounting alignment. No angle is forced to zero when the program starts.

| Slow physical pose, then pause | Expected display |
|---|---|
| Independently level reference plate | Roll and pitch near zero, within calibration/reference errors |
| +Y side raised | Positive roll |
| -Y side raised | Negative roll |
| -X side raised | Positive pitch |
| +X side raised | Negative pitch |

These are the same drone axes used for calibration, not the axis markings printed on the MPU board. We are checking geometric signs before assigning front/back labels to controls.

## Your next test

Upload bench yourself, then close Serial Monitor:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 upload --upload-port COM3
.\qcopter.ps1 record-angles --port COM3 --seconds 90
```

After Enter, the terminal displays MCU roll/pitch in degrees about five times per second. For each table row, position the drone gently, pause for several seconds and note the angle. About 20-30 degrees is enough; a ruler helps if available. Do not aim for ±90 degrees in this first lesson. If a level reference is unavailable, a nonzero initial angle does not itself prove a calibration error.

The raw serial bytes and every selected verified sample save automatically under a new private `artifacts/angle-recordings/` folder. Ctrl+C saves a labeled partial session. Ordinary completion also checks the final header. Original samples during motion are retained; this dataset is not fed into gyro-bias fitting.

## Limits of this observation

During translation/vibration, the accelerometer measures more than gravity and these angles can be wrong. Even acceleration magnitude near g does not prove a stationary pose. The norm warning is a diagnostic, not a validity certificate. Pause before judging each angle. Near free fall or singular poses, angles may be unavailable/unreliable.

This is not a flight-ready attitude estimator. There is no gyro integration, body-frame gyro mapping, Kalman filter, PID, altitude estimate, hover control or yaw estimate. Yaw is unobservable from gravity alone. Output is still a bench stream near/below 100 Hz, not the 500 Hz flight loop. Motor pins remain inputs with pulldowns.

After you verify signs and approximate angles physically, the next lesson can check gyro body-axis direction using known rotations before adding time integration and sensor fusion. The unavailable BMP280 remains deferred. All calibration constants and their boot restoration remain unchanged.
