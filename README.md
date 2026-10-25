# IntelQCopter - small sensor lessons

**Current step: observe gyro XYZ and temperature while preserving the complete accelerometer calibration.** No gyro correction is applied yet. Motor outputs remain disabled.

[Current Arduino main.cpp](firmware/main.cpp) · [Gyro lesson and commands](docs/gyro-observation.md) · [Current plan](docs/PLAN.md) · [Important sources](important_sources/README.md).

Upload bench firmware yourself in PlatformIO, close Serial Monitor, then keep the drone motionless on a fixed support and run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-gyro --port COM3 --seconds 60
```

Progress is visible and original logs save automatically. The PC estimates a candidate bias on the first two-thirds, checks the final third and reports covariance, temperature and drift. Nothing is automatically applied. Bench telemetry is near/below 100 Hz; it is not the future 500 Hz flight loop or final Kalman noise acquisition.

The accelerometer still uses all nine coefficients in `C(sensor-bias)` at arbitrary orientations, with MPU trim and software rounding remainder counted once. [Calibration explanation](docs/full-accelerometer-calibration.md) · [Fitted model](measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md) · [Independent tilted-pose review](measurements/mpu6050/2026-10-25-tilt-validation/report.md).

[First actual MCU XYZ recording](measurements/mpu6050/2026-10-25-first-live-xyz/report.md) and [reset/full-power-cycle checks](measurements/mpu6050/2026-10-25-restart-validation/report.md) verified implementation in those cases. Reference accuracy and flight acceptance remain separate. Gyro changes have build/offline evidence only until you perform the new capture.

[Raw-recorder checkpoint](archive/bench-checkpoints/raw-pose1000/README.md) · [Historical X-bias explanation](docs/bias-x-provenance.html) · [ROS2/Gazebo terminal commands](docs/start-ros-gazebo.md).
