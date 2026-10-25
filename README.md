# IntelQCopter - small software steps

**Current step: review the completed 18-pose calibration/noise analysis, then validate independently measured tilted poses.**

1. [Current report and real-data graphs](measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/report.md).
2. [The single Arduino firmware file](firmware/main.cpp).
3. [The small-step plan](docs/PLAN.md).

The new session contains all 18 recordings (360,000 samples); their hashes, checksums and sequences were rechecked. Repetitions 1-2 produced a provisional calibration; repetition 3 was reserved for validation. Direction errors are about 0.07-1.11 degrees, including pose-reference uncertainty. Some logs have transients and temporal correlation, so no blanket Gaussian/independence claim or final Kalman gain is made.

For recording another session, the PC prompts for Z+, Z-, X+, X-, Y+, Y-, repeated three times. Each recording contains 20,000 FIFO samples at nominal 1000 Hz. Original samples and statistical summaries are saved automatically. Firmware applies no user calibration during these measurements; previous calibrated firmware is preserved at commit a1d3c78. No motor driver or motor output is enabled.

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 build
# Upload bench yourself using PlatformIO, then close Serial Monitor.
.\qcopter.ps1 record-poses --port COM3
```

Arduino / Adafruit MPU6050 / ESP32-S3 / PlatformIO. Allman braces, one statement per line. Source identity, checked FIFO reads, sample numbering and a payload checksum protect the evidence. A successful recording is not physical calibration acceptance. No controller or simulation is added here.

[Calibration across multiple orientations](docs/multi-pose-calibration.md) · [Earlier real-data analysis](docs/precision-2026-10-25.md) · [Important files](important_sources/README.md) · [ROS2/Gazebo commands](docs/start-ros-gazebo.md).

You handle the PCB and simulation. This branch focuses on software lessons. Original evidence and previous implementations remain in local artifacts and Git history. Raw recording folders are excluded from public commits; derived results can be reviewed and published later.

To reproduce the offline analysis with a fresh output directory:

```powershell
.\qcopter.ps1 analyze-poses --session artifacts/pose-recordings/20261025T132707615977Z --output .local/analysis-review-01
```

The analysis requires NumPy/Matplotlib in the command's Python environment. Original raw logs stay local. Current firmware is unchanged; coefficients are not installed or physically accepted. Independent tilt validation and power-cycle checks precede attitude/PID work.
