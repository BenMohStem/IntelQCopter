# Height hold and vertical-speed control — control strategy and limitations

## Reference generation in v2

The controller integrates commanded vertical speed (Xbox left Y) to update relative `heightTarget`; a centered stick requests zero v_z with height correction. The source uses a dead zone (~0.07), normal max speed request about ±0.6 m/s, and height-target clamp about −2 to +8 m in the **virtual** test model.

## Two PID stages

- Height error `z*−z_hat` goes to **height PID** `(0.4,0.005,0.015)` and produces a vertical-speed correction.
- Vertical-speed error `v*−v_hat` goes to **vertical PID** `(1.0,0.06,0.025)` and produces vertical acceleration in m/s².
- Thrust feedforward is `m(g+a*) / cos(tilt)` with lower cos clamp; converted through assumed T_max to virtual motor fractions.

The inner altitude controller is only as reliable as its vertical-speed estimate; differentiating barometer pressure naïvely amplifies noise, so v2 uses the Kalman state `v_z` predicted by accelerometer and corrected by fresh pressure/range observations.

## Why true hover remains difficult

BMP280 pressure responds to atmospheric changes, dynamic pressure from motion and propwash; attitude errors leak horizontal acceleration into vertical dynamics. LiDAR can help near ground but was not in the successful calibration and the current ESP32 RX pin is unverified. No absolute height/position guarantee is possible from BMP alone. Hover with actual motors is not demonstrated.
