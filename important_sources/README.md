# Current important files

1. [main.cpp - one active Arduino firmware](../firmware/main.cpp).
2. [Current gravity roll/pitch lesson](../docs/gravity-tilt-observation.md).
3. [Current plan](../docs/PLAN.md).
4. [Full accelerometer math](../docs/full-accelerometer-calibration.md).
5. [Gyro bias and boot restoration](../docs/gyro-bias-application.md).
6. [Actual gyro reset/power-cycle evidence](../measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md).

BMP280 deferred until available. The new angle lesson is prepared, not physically tested. Calibration constants are unchanged; no motor output. Tools live in tools/calibration; original logs remain private. These links point to canonical files instead of duplicate source copies.

[Assistant-run stationary test: measured results and covariance/tilt derivation](../measurements/mpu6050/2026-10-26-assistant-stationary-test/report.md). Actual board remains V2; angles in this report are PC replay. No upload or calibration changes.

[Sampling-rate limits and three assistant-run repeatability tests](../measurements/mpu6050/2026-10-26-sampling-and-repeatability/report.md): 15,003 actual samples at 83.33 Hz. MPU6050 cannot provide 10 kHz fresh measurements. No high-rate firmware or calibration change was made.
