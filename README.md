# IntelQCopter â€” small software steps

**Current lesson: step04 - check and initialize the MPU6050.**

Open these in order:

1. [The small main.cpp](firmware/main.cpp).
2. [Step04 explanation](docs/step-04.md), building on [step03](docs/step-03.md).
3. [The revised plan](docs/PLAN.md).

Arduino framework, ESP32-S3, PlatformIO. One firmware file. Braces on separate lines; one statement per line. No sensors, calibration, Xbox, PID or ROS2 in the first lesson. Motor signal pins are inputs, with no motor driver.

You handle the PCB. This branch focuses on software. The earlier larger diagnostics are preserved in [the archive](archive/before-small-lessons/README.md) and Git history, not included in the active build.

**Step04 is prepared for your own upload and test.** The user attempted step01 upload; a successful run has not yet been recorded. No physical acceptance is claimed.

To compile when you want:

```powershell
cd C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise
.\qcopter.ps1 build
```

[PlatformIO configuration](platformio.ini) Â· [Important links](important_sources/README.md) Â· [Advanced validation/repair workflow](docs/validation.md).

You build the simulation too. [Commands to open ROS2, RViz and Gazebo yourself](docs/start-ros-gazebo.md).
