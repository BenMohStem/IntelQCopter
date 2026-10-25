# IntelQCopter - small MPU6050 lessons

**Current step: observe gravity-based roll and pitch from the calibrated accelerometer.** BMP280 is unavailable and deferred. Accelerometer and gyro correction constants are unchanged. No PID, altitude control or motor output is enabled.

[Single Arduino main.cpp](firmware/main.cpp) · [Current angle lesson](docs/gravity-tilt-observation.md) · [Plan](docs/PLAN.md) · [Important sources](important_sources/README.md).

Upload bench yourself, close Serial Monitor, then run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 upload --upload-port COM3
.\qcopter.ps1 record-angles --port COM3 --seconds 90
```

Show level and four gentle side-raised poses, pausing at each. The terminal displays actual MCU angles and saves originals automatically. These gravity-only angles are meaningful when stationary; they are not yet a fused flight attitude.

[Gyro reset/power-cycle evidence](measurements/mpu6050/2026-10-26-gyro-restart-validation/report.md) verified 5,003 samples and restored targets [-6,9,27] in the two operator-reported cases. Small residuals remain; no zero-drift claim.

[Five-minute corrected gyro results](measurements/mpu6050/2026-10-26-corrected-gyro-validation/report.md) · [Measured gyro bias](measurements/mpu6050/2026-10-25-three-gyro-runs/report.md) · [Gyro trim explanation](docs/gyro-bias-application.md) · [Accelerometer math](docs/full-accelerometer-calibration.md) · [Tilt calibration review](measurements/mpu6050/2026-10-25-tilt-validation/report.md) · [Raw source checkpoint](archive/bench-checkpoints/raw-pose1000/README.md) · [ROS2/Gazebo terminal commands](docs/start-ros-gazebo.md).

[Assistant-run stationary test: measured results and covariance/tilt derivation](measurements/mpu6050/2026-10-26-assistant-stationary-test/report.md). Actual board remains V2; angles in this report are PC replay. No upload or calibration changes.
