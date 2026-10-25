# IntelQCopter - small software steps

**Current step: collect 18 one-minute MPU6050 recordings, before fitting a new calibration.**

1. [Flash once, then record all 18 poses automatically](docs/record-18-poses.md).
2. [The single Arduino firmware file](firmware/main.cpp).
3. [The small-step plan](docs/PLAN.md).

The PC prompts for Z+, Z-, X+, X-, Y+, Y-, repeated three times. Each recording contains 60,000 FIFO samples at nominal 1000 Hz. Original samples and statistical summaries are saved automatically. Firmware applies no user calibration during these measurements; previous calibrated firmware is preserved at commit a1d3c78. No motor driver or motor output is enabled.

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 build
# Upload bench yourself using PlatformIO, then close Serial Monitor.
.\qcopter.ps1 record-poses --port COM3
```

Arduino / Adafruit MPU6050 / ESP32-S3 / PlatformIO. Allman braces, one statement per line. Source identity, checked FIFO reads, sample numbering and a payload checksum protect the evidence. A successful recording is not physical calibration acceptance. No controller or simulation is added here.

[Calibration across multiple orientations](docs/multi-pose-calibration.md) · [Earlier real-data analysis](docs/precision-2026-10-25.md) · [Important files](important_sources/README.md) · [ROS2/Gazebo commands](docs/start-ros-gazebo.md).

You handle the PCB and simulation. This branch focuses on software lessons. Original evidence and previous implementations remain in local artifacts and Git history. Raw recording folders are excluded from public commits; derived results can be reviewed and published later.
