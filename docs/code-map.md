# Active code and evidence map

| Domain | Active source |
|---|---|
| Flight control, calibration, estimators, battery math | firmware/src/flight_controller.c |
| Public platform boundary | firmware/include/fc_api.h |
| I²C, Xbox, scheduling, telemetry and NVS | firmware/src/esp32_platform.cpp |
| Xtensa CPU replay | simulation/emulation/ |
| Mathematical plant/gain calculation | tools/model/model_lti.py |
| Knowledge validation/search | tools/knowledge/ |
| Offline guide rendering | tools/docs/ |
| UART CSV plotting | tools/reports/ |
| PlatformIO dependency adaptation | tools/platformio/ |
| Historical snapshot index and cache recovery | archive/ and tools/archive/ |

One C file holds flight math, grouped into six numbered sections. C++ handles
existing hardware library interfaces. The emulator compiles that same C source;
it does not maintain a separate flight-controller implementation.

`project.json` declares domains and commands. Run `qcopter.ps1 help` to list them.
Open IntelQCopter.code-workspace for the active ESP32 tree, or the workspace file
in the sibling QCopter directory to see both architectures.

Build products and search indexes live in ignored build/. Historical duplicate
payloads live in ignored .cache/. Machine-specific runtime paths live in .local/.
Original execution logs and failed runs stay in versioned artifacts/.
The active worktree is QCopter-ESP32-Xbox, a sibling of QCopter; neither checkout
is inside a disposable build directory.
