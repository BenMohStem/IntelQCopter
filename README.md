# IntelQCopter - small software steps

**Current lesson C5a: only accelerometer X bias, stored in the MPU register.**

1. [Read the short lesson and bench procedure](docs/bench-x-bias.md).
2. [Open the single Arduino main.cpp](firmware/main.cpp).
3. [Follow the updated plan](docs/PLAN.md).

The provisional bias is 1.308683406 m/s² in sensor X. On boot the programme checks the recorded baseline -2074, writes target -2348 and verifies readback. It applies no software bias again. Y/Z, scale/mounting, gyro and flight control are later lessons. Motor outputs stay disabled.

Build/upload bench yourself in PlatformIO, close Serial Monitor, then keep the drone fixed on a support and run:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 record-bias-x --port COM3
```

It saves raw/corrected/raw intervals automatically (about 66 seconds total). Expected X decrease: 1.312022510 m/s². This is a roughly 10 Hz bench-reporting check; it does not replace the earlier 1000 Hz noise recordings or validate a flight loop. Physical results remain pending. You control all uploads.

[Preserved raw-recorder checkpoint](archive/bench-checkpoints/raw-pose1000/README.md) · [Six-pose analysis](measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md) · [Tilt review](measurements/mpu6050/2026-10-25-tilt-validation/report.md) · [Important files](important_sources/README.md).

The original 18-pose and four-tilt recording commands require the raw-recorder source/firmware, not this partially calibrated lesson. Raw logs and failed attempts stay local; reviewed derived results are published. Mass/inertia parameters are already saved for the later simulation phase. PCB and Gazebo construction remain yours; [terminal setup commands](docs/start-ros-gazebo.md) are available.

[Where 1.312 m/s² comes from: complete calculation report](docs/bias-x-provenance.html) · [LaTeX Markdown source](docs/bias-x-provenance.md).
