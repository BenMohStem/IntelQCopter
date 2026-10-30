# IntelQCopter — stepwise ESP32 flight controller

**Next milestone: live roll/pitch estimation, followed by visible PID/mixer outputs with motors disabled.** Existing calibration is frozen for development; no new calibration campaign is requested.

[Current roadmap and stopping conditions](docs/PLAN.md) · [Important sources](important_sources/README.md) · [Project knowledge](knowledge/README.md)

The ESP32 currently runs the [Xbox/IMU measurement harness](lessons/xbox-test-menu/main.cpp). The separate [calibrated development firmware](firmware/main.cpp) contains a preserved user coefficient edit awaiting review. They are different source versions; do not assume generic upload/record commands select the installed harness.

We have actual IMU acquisition, calibration evidence and Xbox input observations. Live attitude acceptance, this branch's PID/mixer, motor validation and altitude control remain to be completed. The target is1000Hz; throughput observations are not a hard full-controller deadline guarantee. Physical motor outputs remain disabled. BMP280 is unavailable, so fixed-altitude flight is not ready.

Use `qcopter.ps1 help` for registered commands; the roadmap is the authoritative work order. Historical calibration, timing, simulation, PCB and report artifacts are preserved, not a list of steps to repeat. The user designs the PCB and Gazebo environment.

[Latest actual rotation review](measurements/mpu6050/2026-10-30-rotation-review/README.md) · [Input observations](measurements/software/2026-10-30-input-results/README.md) · [Prior full plan](archive/plan-history/2026-10-30-before-visible-milestones.md)
