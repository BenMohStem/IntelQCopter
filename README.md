# IntelQCopter - full XYZ calibration bench

**Current main.cpp applies the complete three-axis accelerometer correction to every sample:** drone = C × (sensor − bias), with MPU register trims and software rounding remainders accounted for exactly once.

[Read the full calibration guide](docs/full-accelerometer-calibration.md) · [Single Arduino main.cpp](firmware/main.cpp) · [Current plan](docs/PLAN.md).

All XYZ readings come from one coherent six-byte sensor burst. The matrix includes all nine fitted coefficients and works at arbitrary orientations within the model/range. The code does not zero the starting pose or force the vector length to g. The previous X-only lesson remains in Git history; no separate single-axis bench sequence is required by this implementation.

Build/upload bench yourself in PlatformIO, close Serial Monitor, then run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-accel --port COM3 --seconds 30
```

The PC displays X/Y/Z and norm live, saves all emitted samples and recomputes the matrix for consistency. No fixed pose sequence is imposed. Original logs stay local in artifacts/accel-recordings. During movement, real acceleration changes the vector and its norm. Telemetry is a bench stream near/below 100 Hz; this is not yet the 500 Hz flight loop.

The provisional coefficients came from training repetitions 1-2, with repetition 3 and later tilted records held out. [Six-pose analysis](measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md) · [Tilt review](measurements/mpu6050/2026-10-25-tilt-validation/report.md). Reference accuracy was approximately ±1 degree by operator estimate. Compile/math checks do not establish absolute accuracy or reset/power-cycle behavior. Motor outputs remain disabled; no upload has been performed by the assistant.

[Original raw-recorder checkpoint](archive/bench-checkpoints/raw-pose1000/README.md) · [Historical X-bias value report](docs/bias-x-provenance.html) · [Important sources](important_sources/README.md) · [ROS2/Gazebo commands](docs/start-ros-gazebo.md). Raw and X-only recording commands belong to their source/protocol checkpoints, not this active firmware.
