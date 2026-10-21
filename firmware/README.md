# Pico firmware

- flight_controller/flight_controller.c — RP2350 flight runtime, estimation/control/calibration.
- handler_controller/handler_controller.c — RP2040 handheld runtime and input/protocol scaffolding.
- Each runtime stays in one C file; existing tests stay beside its subproject.

Root CMake selects the target. Use qcopter.ps1 build-flight or build-handler.
No control code or gain was changed by the 2026-10-21 directory reorganization.
Physical motor arming/output remains disabled.
